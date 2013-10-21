import cv2
import numpy as np
import os, os.path

class colorDetect(object):
    #public vars
    image = None
    colorMin = (0, 0, 0)
    colorMax = (255, 255, 255)
    boxes = []
    
    #private varibles
    _threshed = None
    _contoured = None
    
    #constructor takes a cv image, and the min and max color ranges in hsl range
    def __init__(self, image=None, min=(0, 0, 0), max=(255, 255, 255)):
        if not image == None:
            self.image = image.copy()
        if len(min) == 3 and len(max) == 3:
            self.colorMin = min
            self.colorMax = max
        else:
            raise TypeError("Min and Max must be truples of length 3")
            
    """
    " Public Methods
    """
    #helper function that threshold an image that is in the spectum set for the object
    def thresh(self, image):
        return self._thresh(image)
        
    #helper function for detecting objects of a curtain color
    def detect(self):
        self._thresh()
        return self._contour()
    
    """
    " Private Methods
    """
    #coverts the image to a binary image by setting any value in the
    #threshold to 1 and anything else to 0
    def _thresh(self, image=None):
        if image==None:
            frameImage = self.image
        else:
            frameImage = image
        #convert frame to HSV to make it easier to work with
        bwImage = cv2.cvtColor(frameImage, cv2.COLOR_BGR2HSV)
        #use your custom function to filter by color range
        lowerRed = np.array(self.colorMin)
        upperRed = np.array(self.colorMax)

        imageThresh = cv2.inRange(bwImage, lowerRed, upperRed)
        if image == None:
            self._threshed = imageThresh
        return imageThresh
        
    #finds the contour of the binary image
    def _contour(self, threshImage = None):
        if threshImage==None:
            image = self._threshed
        else:
            image = threshImage
        #make a copy so that the contour get doesnt mess stuff up 
        countImage = image.copy()
        #get the contours from the treshold image copy
        contours,hierarchy = cv2.findContours(countImage, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        #foreach contour return an array of (x, y, width, height)
        boxes = [] 
        for cnt in contours:
            #get rid of really small boxes
            if cv2.contourArea(cnt) > 50:
                #get a simple bounding rect
                boxes.append(cv2.boundingRect(cnt))
        if threshImage == None:
            self.boxes = boxes
        return boxes
    """
    #function that will take our rectangle data and save what was stored there
    def _slice(self):
        image = self.image
        boxes = self.boxes
        files = []
        fileCount = len([name for name in os.listdir('.') if os.path.isfile(name)])
        for box in boxes:
            #sub image= img[y:y+h, x:x+w]
            sub_image = image[box[1]:box[1]+box[3], box[0]:box[0]+box[2]]
            path = os.path.join("objects",str(fileCount)+".jpg")
            cv2.imwrite(path, sub_image)
            files.append(path)
            fileCount += 1
        return files
    """
            
        
