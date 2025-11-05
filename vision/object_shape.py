import cv2
import numpy as np

cap = cv2.VideoCapture(0)

saved = False 

while True:
    ret, frame = cap.read()
    if not ret:
        break

    frame = cv2.flip(frame, 1)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blurred, 10, 50)

    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    display = frame.copy()

    for contour in contours:
        area = cv2.contourArea(contour)
        if area > 1000:
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(display, (x, y), (x+w, y+h), (0, 255, 0), 2)

            if not saved:
                
                np.save("bottlecap_contour.npy", contour)

                cropped = frame[y:y+h, x:x+w]
                cv2.imwrite("bottlecap_image.png", cropped)

                print("Bottle cap shape saved.")
                saved = True

    cv2.imshow("Frame", display)
    cv2.imshow("Edges", edges)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()