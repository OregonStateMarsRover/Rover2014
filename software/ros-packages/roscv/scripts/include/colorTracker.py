
class colorTracker(object):
    objectList = [] 
    image = None
    #searches the image and finds the cords of the match returns it as an array of
    #[x,y,w,h] or false if not found it will then add the match to the tracking info
    #for the object 
    def search(self, image):
        pass
    #searches the image and replaces any matches with black boxes it will then add
    #the match to the tracking info for the object 
    def filter(self, image):
        
    def addTrack(self, index, x, y):
        self.objectList[i] = 
    
    #takes an array of objects as a parameter and adds it to the internal list
    def loadObjects(self, objects):
        self.objectList += objects
        
    def clearObjects(self):
        self.objectList = []