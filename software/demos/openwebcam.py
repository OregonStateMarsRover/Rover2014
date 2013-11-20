import cv2

cv2.namedWindow("left", cv2.CV_WINDOW_AUTOSIZE)

cam = cv2.VideoCapture(0)
while True:
    _,frame = cam.read()
    cv2.imshow("left", frame)
    press = cv2.waitKey(1)
    if  press == 1048603 or press == 27 :
        break