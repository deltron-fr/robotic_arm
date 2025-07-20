import cv2
import numpy as np
from ikpy.chain import Chain
import serial
import time


arm_chain = Chain.from_urdf_file("robot.urdf")

last_grasp_time = 0
grasp_cooldown = 5

ser = serial.Serial('COM3', 9600)
time.sleep(2) 

cap = cv2.VideoCapture(0)

reference_shapes = {
    "bottlecap": np.load("bottlecap_contour.npy"),
    "cup": np.load("cup_contour.npy"),
    "spoon": np.load("spoon_contour.npy")
}

def detect_labeled_shape(frame, reference_shapes):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blurred, 10, 50)

    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    best_label = None
    best_score = float('inf')
    best_centroid = None

    for contour in contours:
        area = cv2.contourArea(contour)
        if area > 1000:
            for label, ref_contour in reference_shapes.items():
                score = cv2.matchShapes(contour, ref_contour, cv2.CONTOURS_MATCH_I1, 0.0)
                if score < 0.1 and score < best_score:
                    M = cv2.moments(contour)
                    if M["m00"] != 0:
                        cX = int(M["m10"] / M["m00"])
                        cY = int(M["m01"] / M["m00"])
                        best_score = score
                        best_centroid = (cX, cY)
                        best_label = label
                        best_contour = contour

    return best_centroid, best_label, best_contour, edges


def pixel_to_world(cX, cY, frame_width, frame_height, real_world_width_cm, real_world_height_cm):
    real_x = (cX / frame_width) * real_world_width_cm
    real_y = (cY / frame_height) * real_world_height_cm
    return real_x, real_y

def compute_joint_angles(target):
    np.set_printoptions(suppress=True, precision=10)

    joint_angles = np.degrees(arm_chain.inverse_kinematics(target))

    return list(map(lambda x: round(x, 2), joint_angles))

def send_angles_to_arduino(joint_angles):
    command = ",".join(str(int(angle)) for angle in joint_angles) + '\n'
    ser.write(command.encode())
    print("Sent:", command.strip())

    start_time = time.time()
    while True:
        if ser.in_waiting:
            response = ser.readline().decode().strip()
            if response == "OK":
                break
        if time.time() - start_time > 3:  
            print("Timeout waiting for OK from Arduino.")
            break

HOME = compute_joint_angles(x=0, y=0, z=15)
DROP_US = compute_joint_angles(x=15, y=10, z=10)
DROP_UK = compute_joint_angles(x =20, y=10, z=10)

def pick_and_place(grasp_x, grasp_y, grasp_z, item):
    PRE_GRASP = compute_joint_angles(grasp_x, grasp_y, grasp_z + 5)
    GRASP = compute_joint_angles(grasp_x, grasp_y, grasp_z)
    LIFT = compute_joint_angles(grasp_x, grasp_y, grasp_z + 10)

    send_angles_to_arduino(HOME)
    time.sleep(1)

    send_angles_to_arduino(PRE_GRASP)
    time.sleep(1)

    send_angles_to_arduino(GRASP)
    time.sleep(1)

    ser.write(b'CLOSE\n') 
    time.sleep(1)

    send_angles_to_arduino(LIFT)
    time.sleep(1)

    if item == "bottlecap":
        send_angles_to_arduino(DROP_US)
        time.sleep(1)
    
    else:
        send_angles_to_arduino(DROP_UK)
        time.sleep(1)

    ser.write(b'OPEN\n') 
    time.sleep(1)

    send_angles_to_arduino(HOME)


while True:
    ret, frame = cap.read()
    if not ret:
        break
    frame = cv2.flip(frame, 1)
    frame_height, frame_width = frame.shape[:2]

    centroid, label, contour, edges = detect_labeled_shape(frame, reference_shapes)
    if centroid and (time.time() - last_grasp_time > grasp_cooldown) and contour:
        x, y, w, h = cv2.boundingRect(contour)
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.putText(frame, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        print(f"Detected {label} at {centroid}")
        cX, cY = centroid
        real_x, real_y = pixel_to_world(cX, cY, frame_width, frame_height, 40, 30)
        real_z = 0  

        pick_and_place(real_x, real_y, real_z, label)
        last_grasp_time = time.time()

    cv2.imshow("Edges", edges)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


cap.release()
cv2.destroyAllWindows()