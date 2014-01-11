#!/usr/bin/env python

import numpy as np
import cv2
import math

HUE_MAX = 180

def initCam():
	cam = cv2.VideoCapture(0)
	return cam

def getFrame(cam):
	_,frame = cam.read()
	return frame

def hueDist(image, goal):
	hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
	hue = hsv[...,0]
	hue = hue.astype(np.int)
	#hue = abs(goal - hue)
	hue = abs(HUE_MAX - hue + goal)
	#hue = np.min(hue1, hue2)
	
	hue = hue.astype(np.uint8)
	return hue

def colDist(image, goal):
	HUE_WEIGHT = 1.0
	SAT_WEIGHT = 0.1
	VAL_WEIGHT = 0.1
	hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
	hsv = hsv.astype(np.int)
	d = HUE_WEIGHT * (hsv[...,0] - goal[0]) ** 2 + SAT_WEIGHT * (hsv[...,1] - goal[1]) ** 2 + VAL_WEIGHT * (hsv[...,2] - goal[2]) ** 2
	max_d = np.max(d)
	d = (d * 255) / max_d
	d = d.astype(np.uint8)
	d = 255 - d
	return d

def thresh(image, tol=0.1):
	m = np.max(image)
	n = int(m * (1.0 - tol))
	_,image = cv2.threshold(image, n, 255, cv2.THRESH_BINARY)
	image = 255 - image
	return image

def find_balls(image):
	"""
	image = cv2.GaussianBlur(image, (7, 7), 0)
	k = cv2.getStructuringElement(cv2.MORPH_CROSS, (17,17))
	e = cv2.erode(image, k)
	image = image - e
	"""
	tmp = image.copy()
	conts,_ = cv2.findContours(tmp, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

	circles = []
	best_c = 0
	for c in conts:
		c = c.astype(np.int64)
		area = cv2.contourArea(c)
		if 50 < area < 100000:
			arc = cv2.arcLength(c, True)
			circ = 4*math.pi * area / (arc ** 2)
			if circ > 0.5:
				circles.append(c)
	return circles, image


cv2.namedWindow('image', cv2.WINDOW_NORMAL)
#cv2.namedWindow('1', cv2.WINDOW_NORMAL)
cv2.namedWindow('2', cv2.WINDOW_NORMAL)

orange = (15, 255, 255)
blue = (90, 255, 255)
orange_tol = 0.1
blue_tol = 0.3

cam = initCam()

while True:
	orig_img = getFrame(cam)
	#img1 = colDist(orig_img, (15, 255, 255))
	img2 = colDist(orig_img, (90, 255, 255))
	#img1 = thresh(img1, orange_tol)
	img2 = thresh(img2, blue_tol)
	#img1 = find_balls(img1)
	circles, img2 = find_balls(img2)
	for i in range(len(circles)):
		cv2.drawContours(orig_img, circles, i, (255, 0, 0), 5)
	cv2.imshow('image', orig_img)
	#cv2.imshow('1', img1)
	cv2.imshow('2', img2)
	if cv2.waitKey(33) == 27: break
cv2.destroyAllWindows()
