import cv2

cap = cv2.VideoCapture(0)

# Initialize QR code detector
qr_detector = cv2.QRCodeDetector()

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Detect and decode QR code
    data, points, _ = qr_detector.detectAndDecode(frame)

    if points is not None:
        points = points[0].astype(int)
        # Draw bounding box around QR code
        for i in range(len(points)):
            pt1 = tuple(points[i])
            pt2 = tuple(points[(i + 1) % len(points)])
            cv2.line(frame, pt1, pt2, (0, 255, 0), 2)

        if data:
            print(f"QR Code detected: {data}")
            # Optionally draw the text near the box
            cv2.putText(frame, data, (points[0][0], points[0][1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)

    cv2.imshow("QR Code Detection", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
