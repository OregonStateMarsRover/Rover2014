import cv2


###################################################################################################################################
# Function name: importImages()
# Description: Imports a series of images that will be added to a list of images, all to be used to teach the robot a certain item.
# Returns: List containing the images
###################################################################################################################################

def importImages():

	images = [] 

	for i in range(20):
		string = "images" + str(i + 1) + ".png"
	
		img = cv2.imread(string, cv2.CV_LOAD_IMAGE_COLOR)
		res = cv2.resize(img, (100, 100))
		blur = cv2.GaussianBlur(res, (5,5), 0)

		images.append(blur)		

	return images

images =importImages();

for i in range(20):
	cv2.imshow("image", images[i])
	cv2.waitKey(0)
cv2.DestroyAllWindows()
