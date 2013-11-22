import cv2
import numpy as np

def nothing(n):
    pass

"""
" See the comments in the color filter code since it is exactly the same code
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
" Pass this function a image that has been thresholded it will then 
" overlay any contour boxes over the overlayImage on top of the repective areas
" the color is the color of the overlayed box the tracker is an array of previous
" entries for various colors
"""
    
def contourDetect(threshImage, overlayImage, color=(0,255,0), tracker=None):
    #make a copy so that the contour get doesnt mess stuff up 
    countImage = threshImage.copy()
    #get the contours from the treshold image copy
    contours,hierarchy = cv2.findContours(countImage, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    #foreach contour found draw a box around the area 
    for cnt in contours:
        #get a simple bounding rect
        x,y,w,h = cv2.boundingRect(cnt)
        #get rid of really small boxes
        if cv2.contourArea(cnt) > 1000:
            cv2.rectangle(overlayImage,(x,y),(x+w,y+h),color,10)

def main():
    cv2.namedWindow("setting", cv2.CV_WINDOW_AUTOSIZE)
    cv2.namedWindow("Red", cv2.CV_WINDOW_AUTOSIZE)
    #create a cam that pulls from our cam source
    video = cv2.VideoCapture(0)
    while True:
        #grab a frame
        _, frame = video.read()
        #generate the threshold images
        blueThresh = getThresholdImage(frame, [90,120, 50], [150, 255, 255])
        redThresh = getThresholdImage(frame, [120,100, 60], [255, 255, 255])
        orangeThresh = getThresholdImage(frame, [7,110, 60], [30, 255, 255])
        pinkThresh = getThresholdImage(frame, [110,30, 140], [255, 85, 255])
        
        #generate the contour detection for every color
        contourDetect(blueThresh, frame, (255, 0, 0))
        contourDetect(redThresh, frame, (0, 0, 255))
        contourDetect(orangeThresh, frame, (100, 150, 255))
        contourDetect(pinkThresh, frame, (150, 50, 255))
        
        cv2.imshow("setting", frame)
        cv2.imshow("Blue", blueThresh)
        cv2.imshow("Red", redThresh)
        key = cv2.waitKey(1)
        if key ==  1048603:
            break;
    #"""
main()
