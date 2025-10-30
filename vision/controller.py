# controller.py
import cv2
import numpy as np
import serial
import time
from ikpy.chain import Chain
from ikpy.link import OriginLink

# -------------------------------
# Configuration
# -------------------------------
PORT = 'COM3'
BAUD = 115200                 # faster & smoother than 9600
GRASP_COOLDOWN_S = 5
QR_SIZE_M = 0.04              # physical QR side length in meters (adjust to your tag)
Z_PICK_M = 0.02               # table height relative to robot base (adjust)
Z_PREGRASP_OFFSET_M = 0.06    # hover above grasp
Z_LIFT_OFFSET_M = 0.12
MOVE_TIME_MS = 1600           # default segment duration
HOME_JOINTS_DEG = [0, -30, 60, 0, 30, 0]   # safe, reachable pose
DROP_US_JOINTS_DEG = [30, -20, 50, 15, 20, 0]
DROP_UK_JOINTS_DEG = [-30, -20, 50, -15, 20, 0]
DEFAULT_DROP_JOINTS_DEG = [0, -10, 40, 0, 20, 0]

# If your URDF has 6 actuated joints, this should work.
# If IK returns N joints (including fixed/base), we'll slice to first 6 actuated.
ROBOT_URDF = "robot.urdf"

# -------------------------------
# Init hardware/software
# -------------------------------
print("[Init] Loading kinematic chain...")
arm_chain = Chain.from_urdf_file(ROBOT_URDF)

# Determine joint count and a slice for the first 6 actuated joints.
# ikpy usually returns len(chain.links) angles (with first possibly fixed).
# We assume the last link is the tool; we need 6 DOF joints.
NUM_RETURN = len(arm_chain.links)  # includes fixed/origin links
# Heuristic: drop first (origin) and last (tool) if present, keep 6
# Adjust to your chain if different.
def _slice6(q):
    q = list(np.degrees(q)) if isinstance(q, np.ndarray) else list(q)
    if len(q) >= 8:              # e.g., [fixed] + 6 + [tool]
        q = q[1:7]
    elif len(q) > 6:
        q = q[:6]
    return [round(a, 2) for a in q]

print("[Init] Opening serial...")
ser = serial.Serial(PORT, BAUD, timeout=0.1)
time.sleep(2)

print("[Init] Opening camera...")
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    raise RuntimeError("Camera not available.")

print("[Init] Loading camera calibration (optional)...")
camera_matrix, dist_coeffs = None, None
try:
    camera_matrix = np.load("camera_mtx.npy")
    dist_coeffs = np.load("dist_coeffs.npy")
    print("[Calib] Loaded camera_mtx.npy and dist_coeffs.npy")
except Exception:
    print("[Calib] Not found. Will use pixel scaling fallback.")

qr = cv2.QRCodeDetector()

last_grasp_time = 0.0

# -------------------------------
# Helpers
# -------------------------------
def send_move(joint_angles_deg, duration_ms=MOVE_TIME_MS):
    """
    Send a timed joint move to Arduino. Blocks until Arduino replies 'OK' or timeout.
    """
    if len(joint_angles_deg) != 6:
        raise ValueError("Expected 6 joint angles.")
    payload = ",".join(str(int(round(a))) for a in joint_angles_deg)
    cmd = f"MOVE {payload} T={int(duration_ms)}\n"
    ser.write(cmd.encode())
    # wait for OK
    t0 = time.time()
    while True:
        line = ser.readline().decode().strip()
        if line == "OK":
            return True
        if time.time() - t0 > (duration_ms/1000.0 + 2.0):
            print("[WARN] Timeout waiting for OK:", cmd.strip())
            return False

def send_grip(state):
    """
    state: 'OPEN' or 'CLOSE'
    """
    ser.write(f"GRIP {state}\n".encode())
    # optional: wait for OK so timing aligns
    t0 = time.time()
    while True:
        line = ser.readline().decode().strip()
        if line == "OK":
            return True
        if time.time() - t0 > 2.0:
            print("[WARN] Timeout waiting for OK for GRIP", state)
            return False

def send_home():
    ser.write(b"HOME\n")
    t0 = time.time()
    while True:
        line = ser.readline().decode().strip()
        if line == "OK":
            return True
        if time.time() - t0 > 3.0:
            print("[WARN] Timeout waiting for OK for HOME")
            return False

def build_target_transform(x, y, z):
    """
    Build a 4x4 target transform for IK at position (x,y,z) in meters.
    End-effector pointing down along -Z with neutral wrist. Adjust if needed.
    """
    T = np.eye(4)
    T[0, 3] = x
    T[1, 3] = y
    T[2, 3] = z
    # Simple orientation: identity (adjust if your gripper must face differently)
    return T

def compute_joint_angles_xyz(x, y, z, q_init=None):
    T = build_target_transform(x, y, z)
    if q_init is None:
        q_init = [0.0]*(len(arm_chain.links))
    q = arm_chain.inverse_kinematics(T, initial_position=q_init)
    return _slice6(q)

def choose_drop_joints(decoded_text: str):
    if not decoded_text:
        return DEFAULT_DROP_JOINTS_DEG
    t = decoded_text.strip().upper()
    if "US" in t:
        return DROP_US_JOINTS_DEG
    if "UK" in t:
        return DROP_UK_JOINTS_DEG
    return DEFAULT_DROP_JOINTS_DEG

def robust_ok():
    # Drain any OKs already waiting
    drained = False
    while True:
        line = ser.readline().decode().strip()
        if not line:
            break
        drained = True
    return drained

# Simple pixel→world fallback (planar scaling). Replace with proper homography if needed.
def pixel_to_world_fallback(cX, cY, frame_w, frame_h, width_m=0.40, height_m=0.30):
    wx = (cX / frame_w) * width_m
    wy = (cY / frame_h) * height_m
    return wx, wy

def estimate_qr_pose(points, qr_size_m, camera_matrix, dist_coeffs):
    """
    Estimate pose from 4 image points (2D) of the QR corners.
    Returns position (x,y,z) in meters relative to camera.
    Corner order from cv2.QRCodeDetector: typically [tl, tr, br, bl]
    """
    # Define QR square corners in object frame (Z=0 plane)
    s = qr_size_m
    objp = np.array([
        [-s/2, -s/2, 0],
        [ s/2, -s/2, 0],
        [ s/2,  s/2, 0],
        [-s/2,  s/2, 0],
    ], dtype=np.float32)

    imgp = np.array(points, dtype=np.float32)
    # Ensure order matches. If different, reorder as needed.
    ok, rvec, tvec = cv2.solvePnP(objp, imgp, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok:
        return None
    R, _ = cv2.Rodrigues(rvec)
    pos = tvec.reshape(3)  # in camera frame (meters if your K is meters; usually it is in pixels → tvec in meters)
    return pos, R

# -------------------------------
# Main loop
# -------------------------------
print("[Run] Draining serial...")
robust_ok()

print("[Run] Homing...")
send_home()  # Arduino handles homing to a known set of joint angles
send_move(HOME_JOINTS_DEG, 1200)

while True:
    ret, frame = cap.read()
    if not ret:
        break
    frame = cv2.flip(frame, 1)
    h, w = frame.shape[:2]

    decoded_texts, points_list, _ = qr.detectAndDecodeMulti(frame)
    # decoded_texts is list of strings; points_list is Nx1x4x2 or Nx4x2 depending on OpenCV
    target_found = False
    chosen_text = None
    chosen_pts = None

    if points_list is not None and len(points_list) > 0:
        # pick the QR closest to image center
        cx_img, cy_img = w/2, h/2
        best_d, best_idx = float('inf'), -1
        for i, pts in enumerate(points_list):
            pts = np.squeeze(pts)  # 4x2
            c = pts.mean(axis=0)   # centroid
            d = (c[0]-cx_img)**2 + (c[1]-cy_img)**2
            if d < best_d:
                best_d, best_idx = d, i
        chosen_pts = np.squeeze(points_list[best_idx])
        chosen_text = decoded_texts[best_idx] if decoded_texts and best_idx < len(decoded_texts) else ""

        # Draw box
        p = chosen_pts.astype(int)
        for i in range(4):
            cv2.line(frame, tuple(p[i]), tuple(p[(i+1)%4]), (0,255,0), 2)
        c = p.mean(axis=0).astype(int)
        cv2.circle(frame, tuple(c), 5, (255,0,0), -1)
        if chosen_text:
            cv2.putText(frame, chosen_text, (p[0][0], p[0][1]-10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)

        # Compute world/robot coordinates
        if camera_matrix is not None and dist_coeffs is not None:
            pose = estimate_qr_pose(chosen_pts, QR_SIZE_M, camera_matrix, dist_coeffs)
            if pose is not None:
                (tx, ty, tz), R = pose
                # Convert from camera frame to robot base frame if needed.
                # For now assume camera frame == robot base frame. Replace with extrinsic transform if mounted differently.
                x_m, y_m, z_m = float(tx), float(ty), float(tz)
            else:
                # Fallback if PnP failed
                cx, cy = int(c[0]), int(c[1])
                x_m, y_m = pixel_to_world_fallback(cx, cy, w, h, width_m=0.40, height_m=0.30)
                z_m = Z_PICK_M
        else:
            # No calibration → crude mapping
            cx, cy = int(c[0]), int(c[1])
            x_m, y_m = pixel_to_world_fallback(cx, cy, w, h, width_m=0.40, height_m=0.30)
            z_m = Z_PICK_M

        target_found = True

    cv2.imshow("QR Pick & Place", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    # Plan & execute pick only if cooldown elapsed and target exists
    now = time.time()
    if target_found and (now - last_grasp_time > GRASP_COOLDOWN_S):
        print(f"[Task] Target @ ({x_m:.3f}, {y_m:.3f}, {z_m:.3f}) m | QR='{chosen_text}'")

        # Build waypoints in task space
        pre_z = z_m + Z_PREGRASP_OFFSET_M
        lift_z = z_m + Z_LIFT_OFFSET_M

        # IK for waypoints; reuse last solution as warm start for smoother IK
        q_curr = None

        def ik(x, y, z, q0=None):
            T = build_target_transform(x, y, z)
            q = arm_chain.inverse_kinematics(T, initial_position=(q0 if q0 is not None else [0]*len(arm_chain.links)))
            return _slice6(q), q

        # HOME
        send_move(HOME_JOINTS_DEG, 1000)

        # PRE_GRASP
        pre_deg, q_curr = ik(x_m, y_m, pre_z, q_curr)
        send_move(pre_deg, MOVE_TIME_MS)

        # GRASP
        grasp_deg, q_curr = ik(x_m, y_m, z_m, q_curr)
        send_move(grasp_deg, MOVE_TIME_MS)

        # Close gripper
        send_grip("CLOSE")

        # LIFT
        lift_deg, q_curr = ik(x_m, y_m, lift_z, q_curr)
        send_move(lift_deg, MOVE_TIME_MS)

        # DROP (route by text)
        drop_deg = choose_drop_joints(chosen_text)
        send_move(drop_deg, MOVE_TIME_MS)

        # Open gripper
        send_grip("OPEN")

        # Return home
        send_move(HOME_JOINTS_DEG, 1200)

        last_grasp_time = time.time()

cap.release()
cv2.destroyAllWindows()
