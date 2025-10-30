# controller.py (barebones)
import cv2
import numpy as np
import serial
import time
from ikpy.chain import Chain


PORT = 'COM3'
BAUD = 115200
ESP32_STREAM_URL = "http://192.168.4.1:81/stream"  

QR_SIZE_M = 0.04              # just for reference
Z_PICK_M = 0.02               # table height
Z_PREGRASP_OFFSET_M = 0.06
Z_LIFT_OFFSET_M = 0.12
MOVE_TIME_MS = 1600

HOME_JOINTS_DEG = [0, -30, 60, 0, 30, 0]
DROP_US_JOINTS_DEG = [30, -20, 50, 15, 20, 0]
DROP_UK_JOINTS_DEG = [-30, -20, 50, -15, 20, 0]
DEFAULT_DROP_JOINTS_DEG = [0, -10, 40, 0, 20, 0]

ROBOT_URDF = "robot.urdf"


arm_chain = Chain.from_urdf_file(ROBOT_URDF)
ser = serial.Serial(PORT, BAUD, timeout=0.1)
time.sleep(2)

cap = cv2.VideoCapture(ESP32_STREAM_URL, cv2.CAP_FFMPEG)
if not cap.isOpened():
    raise RuntimeError("ESP32-CAM stream not available.")

qr = cv2.QRCodeDetector()

def _slice6(q):
    q = list(np.degrees(q)) if isinstance(q, np.ndarray) else list(q)
    if len(q) >= 8:  
        q = q[1:7]
    elif len(q) > 6:
        q = q[:6]
    return [round(a, 2) for a in q]


def send_move(joint_angles_deg, duration_ms=MOVE_TIME_MS):
    payload = ",".join(str(int(round(a))) for a in joint_angles_deg)
    cmd = f"MOVE {payload} T={int(duration_ms)}\n"
    ser.write(cmd.encode())
    ser.flush()
    while True:
        line = ser.readline().decode().strip()
        if line == "OK": break

def send_grip(state):
    ser.write(f"GRIP {state}\n".encode())
    while True:
        line = ser.readline().decode().strip()
        if line == "OK": break

def send_home():
    ser.write(b"HOME\n")
    while True:
        line = ser.readline().decode().strip()
        if line == "OK": break

def build_target_transform(x, y, z):
    T = np.eye(4)
    T[0, 3] = x; T[1, 3] = y; T[2, 3] = z
    return T

def compute_joint_angles_xyz(x, y, z, q_init=None):
    T = build_target_transform(x, y, z)
    if q_init is None:
        q_init = [0.0]*len(arm_chain.links)
    q = arm_chain.inverse_kinematics(T, initial_position=q_init)
    return _slice6(q)

def choose_drop_joints(decoded_text: str):
    t = (decoded_text or "").upper()
    if "Enugu" in t: return DROP_US_JOINTS_DEG
    if "Lagos" in t: return DROP_UK_JOINTS_DEG
    return DEFAULT_DROP_JOINTS_DEG

def pixel_to_world(cX, cY, frame_w, frame_h, width_m=0.40, height_m=0.30):
    wx = (cX / frame_w) * width_m
    wy = (cY / frame_h) * height_m
    return wx, wy


send_home()
send_move(HOME_JOINTS_DEG, 1200)

last_grasp_time = 0
COOLDOWN_S = 5

while True:
    ret, frame = cap.read()
    if not ret: break
    h, w = frame.shape[:2]

    decoded_texts, points_list, _ = qr.detectAndDecodeMulti(frame)
    target_found = False

    if points_list is not None and len(points_list) > 0:
        
        cx_img, cy_img = w/2, h/2
        best_d, best_idx = float('inf'), -1
        for i, pts in enumerate(points_list):
            pts = np.squeeze(pts)
            c = pts.mean(axis=0)
            d = (c[0]-cx_img)**2 + (c[1]-cy_img)**2
            if d < best_d: best_d, best_idx = d, i
        chosen_pts = np.squeeze(points_list[best_idx])
        chosen_text = decoded_texts[best_idx] if decoded_texts else ""
        c = chosen_pts.mean(axis=0).astype(int)

        x_m, y_m = pixel_to_world(c[0], c[1], w, h)
        z_m = Z_PICK_M
        target_found = True

        cv2.putText(frame, chosen_text, (c[0], c[1]-10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)
        cv2.circle(frame, tuple(c), 5, (255,0,0), -1)

    cv2.imshow("QR Pick & Place", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    now = time.time()
    if target_found and (now - last_grasp_time > COOLDOWN_S):
        print(f"[Task] Pick {chosen_text} @ ({x_m:.3f}, {y_m:.3f}, {z_m:.3f})")

        pre_deg = compute_joint_angles_xyz(x_m, y_m, z_m+Z_PREGRASP_OFFSET_M)
        grasp_deg = compute_joint_angles_xyz(x_m, y_m, z_m)
        lift_deg = compute_joint_angles_xyz(x_m, y_m, z_m+Z_LIFT_OFFSET_M)
        drop_deg = choose_drop_joints(chosen_text)

        send_move(HOME_JOINTS_DEG, 1000)
        send_move(pre_deg)
        send_move(grasp_deg)
        send_grip("CLOSE")
        send_move(lift_deg)
        send_move(drop_deg)
        send_grip("OPEN")
        send_move(HOME_JOINTS_DEG, 1200)

        last_grasp_time = time.time()

cap.release()
cv2.destroyAllWindows()
