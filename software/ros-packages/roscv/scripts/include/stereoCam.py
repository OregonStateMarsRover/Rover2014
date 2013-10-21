import cv2
import numpy as np

"""
" file: stereoCam.py
" By: Lane Breneman    
" Date: 8/29/2013
"
" This class will take two cameras and will read a feed from them simultaneously
" this will help to keep the frames synced hopefully. i will also add features like
" point clouds for 3d vision
"""

class stereoCam(object):
    #left camera
    left = None
    #right camera
    right = None
    
    """
    " Takes two arguments left and right, they should be the number of the camera 
    " that you want to load, for example 1 would be /dev/video1
    """
    def __init__(self, left=1, right=2):
        self.left = cv2.VideoCapture(left)
        self.right = cv2.VideoCapture(right)
        #give a more precise error
        if not (self.right.isOpened() or self.left.isOpened()):
            raise Exception("Cameras cannot load")
    
    def grab_frame(self):
        self.right.grab()
        self.left.grab()
        _, rightFrame = self.right.retrieve()
        _, leftFrame = self.left.retrieve()
        return (leftFrame, rightFrame)
    
    
   