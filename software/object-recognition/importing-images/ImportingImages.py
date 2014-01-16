import cv2


###################################################################################################################################
# Function name: importImages()
# Description: Imports a series of images that will be added to a list of images, all to be used to teach the robot a certain item.
# Returns: List containing the images
###################################################################################################################################

def importImages():

	images[] = [][]

	for i in range(20):
		string = "tennisball" + (i + 1) + ".jpg"
	
		img = cv2.imread(string, cv2.CV_LOAD_IMAGE_COLOR)
		res = cv2.resize(img, ((1/2)*width, (1/2)*height))
		blur = cv2.gaussianBlur(res, (5,5), 0)

		images[i] = blur		

	return images
