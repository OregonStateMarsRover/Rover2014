import cv2
import numpy as np
import os, os.path

class imageSearch(object):
    """
    " Search an image for a list of template images
    """
    def compare(self, opencv_needle, filelist):
        image = cv2.cvtColor(opencv_needle, cv2.COLOR_BGR2GRAY)
        matches = list()
        #go through every image on the list and find its location
        for filename in filelist:
            template = cv2.imread(filename, 0)
            point = self.match(image, template)
            width = template.shape[0]
            height = template.shape[1]
            matches.append([point[0], point[1], width, height])
            #change to pass a converted surf object
            
        return matches
            
    """
    " Match needle inside the haystack image 
    """
    def match(self, image, template):
        result = cv2.matchTemplate(image, template, cv2.TM_CCORR_NORMED)
        normal = cv2.normalize(result, result, 0, 1, cv2.NORM_MINMAX, -1)
        minVal, maxVal, minLoc, maxLoc = cv2.minMaxLoc(result)
        return maxLoc