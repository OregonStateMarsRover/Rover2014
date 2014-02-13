#!/usr/bin/python2
import cv2
import os
import sys

###################################################################################################################################
# Function name: importImages()
# Description: Imports a series of images that will be added to a list of images, all to be used to teach the robot a certain item.
# Returns: List containing the images
###################################################################################################################################

def importImages():

	images = []
	if sys.argv[1] == "surf":
		f = open("surf_desc.dat", "w")
	files = os.listdir("images/hockeypuck")
	for i, n in enumerate(files):	
		path = os.path.join("images/hockeypuck", n)
		if not (os.path.isfile(path) or ".png" in n):
			continue
		
		if sys.argv[1] == "color":
			img = cv2.imread(path, cv2.CV_LOAD_IMAGE_COLOR)
			res = cv2.resize(img, ((1/2)*width, (1/2)*height))
			blur = cv2.gaussianBlur(res, (5,5), 0)
			images[i] = blur
		elif sys.argv[1] == "surf":
			print "starting image:", i		
			img = cv2.imread(path, cv2.CV_LOAD_IMAGE_GRAYSCALE)
			surf = cv2.SURF(500)
			kp, desc = surf.detectAndCompute(img, None)	
			for z, pt in enumerate(kp):
				f.write("%d " % i)
				f.write("%d %d" % pt.pt)
				descs = ""	
				for x in desc[z]:
					descs += " %f" % x
				descs += "\n"
				f.write(descs)	
	return images

if __name__ == '__main__':
	import timeit
	time = timeit.timeit(importImages, number=1)
	print "Ran Over Images In:", time
