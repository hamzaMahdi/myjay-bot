import cv2
pipeline = "rtspsrc location=\"rtsp://login:password@0.0.0.0:5000/\" ! rtph264depay ! h264parse ! omxh264dec ! nvvidconv ! video/x-raw, format=(string)BGRx! videoconvert ! appsink"
capture = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

while capture.isOpened():
    res, frame = capture.read()
    cv2.imshow("Video", frame)
    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        break
capture.release()
cv2.destroyAllWindows()
