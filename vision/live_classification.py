import cv2
import numpy as np


cap = cv2.VideoCapture(0)
saved_contour = np.load("bottlecap_contour.npy")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blurred, 10, 50)

    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    display = frame.copy()

    for contour in contours:
        area = cv2.contourArea(contour)
        if area > 1000:
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

            match_score = cv2.matchShapes(contour, saved_contour, cv2.CONTOURS_MATCH_I1, 0.0)
            if match_score < 0.1: 
                print("Bottle cap detected!")

    cv2.imshow("Display", display)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()