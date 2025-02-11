import cv2
import cv2.legacy

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 960)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 540)
#cap.set(cv2.CAP_PROP_BRIGHTNESS, 10000000)

if not cap.isOpened():
    print("Error opening video stream or file")
    exit()

while True:
    ret, frame = cap.read()

    if not ret:
        print('Cant receive frame, exiting...')
        break

    cv2.imshow('frame', frame)
    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()