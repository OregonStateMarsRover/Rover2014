import cv2
import numpy as np

#this here because we need a callback for the function be we dont really use it
def nothing(n):
    pass

"""
" This function will get a threshhold image, which is an image where you remove
" all but the color specturm we care about and then turn it black and white
"""
def getThresholdImage(frame, lower, upper):
    #convert frame to HSV to make it easier to work with
    frameImage = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    #use your custom function to filter by color range
    #default :[110,50,50] [130,255,255]
    lowerRed = np.array(lower)
    upperRed = np.array(upper)
    imageThresh = cv2.inRange(frameImage, lowerRed, upperRed)
    #bitwise and adds the color back
    #imageThresh = cv2.bitwise_and(frameImage, frameImage, mask=imageThresh)
    return imageThresh
    


"""
" This is some vary ugly code for filtering the color using trackbars so that you
" can easily find the color spectrum that you want
"""
def main():
    cv2.namedWindow("setting", cv2.CV_WINDOW_AUTOSIZE)
    cv2.namedWindow("Red", cv2.CV_WINDOW_AUTOSIZE)
    #create a cam that pulls from our cam source
    video = cv2.VideoCapture(0)
    #[90,50, 50], [120, 255, 255] blue 
    #[0,60, 60], [30, 255, 255] red
    #orange [7,90, 60], [30, 255, 255]
    cv2.createTrackbar('h-min','setting',7,255,nothing)
    cv2.createTrackbar('h-max','setting',30,255,nothing)
    cv2.createTrackbar('s-min','setting',90,255,nothing)
    cv2.createTrackbar('s-max','setting',255,255,nothing)
    cv2.createTrackbar('l-min','setting',60,255,nothing)
    cv2.createTrackbar('l-max','setting',255,255,nothing)
    while True:
        #grab a frame
        _, frame = video.read()
        hmin = cv2.getTrackbarPos('h-min','setting')
        hmax = cv2.getTrackbarPos('h-max','setting')
        smin = cv2.getTrackbarPos('s-min','setting')
        smax = cv2.getTrackbarPos('s-max','setting')
        lmin = cv2.getTrackbarPos('l-min','setting')
        lmax = cv2.getTrackbarPos('l-max','setting')
        redThresh = getThresholdImage(frame, [hmin,smin, lmin], [hmax, smax, lmax])
        cv2.imshow("window 1", frame)
        cv2.imshow("Red", redThresh)
        key = cv2.waitKey(1)
        if key ==  1048603:
            break;
main()
