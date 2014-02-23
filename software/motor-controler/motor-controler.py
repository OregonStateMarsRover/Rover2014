import itertools #Used to break apart string

DEFAULTSPEED = 20 #Should be a integer

class control_class:
    def __init__(self, port):
        self.port = port
        self.speed = DEFAULTSPEED
    
    def setspeed(self, raw_speed):
        self.speed = 10 * int(raw_speed)
        print("Set Speed: " + raw_speed)

    def _waitr(self):
        #Wait for R response from controller
        #use self.port
        pass

    def sendstatus(self, status):
        #not to sure about this one
        pass


#Specific Class Structure
class motor_control_class(control_class):
    def checkarmstate(self):
        #This one could be a problem, because it is out of the scope of the class
        pass

    def halt(self):
        #stop the robot!
        pass

    def forward(self, raw_distance):
        checkarmstate()
        distance = 100 * int(raw_distance)
        self._waitr()
        print("Distance: " + raw_distance)
        

    def backward(self, raw_distance):
        checkarmstate()
        distance = 100 * int(raw_distance)
        self._waitr()
        print("Distance Backward: " + raw_distance)

    def rotate(self, raw_degrees):
        checkarmstate()
        degrees = int(raw_degrees)
        self._waitr()
        print("Rotate: " + raw_degrees)
        
class arm_control_class(control_class):
    def base_rotate(self, raw_degrees):
        degrees = 100 * int(raw_degrees)
        self._waitr()
        print("Rotate Base to " + raw_degrees)

def registerdevices():
    dictionary = dict()
    dictionary['motor'] = "/dev/ptt2/"
    dictionary['arm'] = "/dev/ptt5/"
    return dictionary

#Hook for pause function of robot, returns 1 if robot is paused
def checkforpause():
    #get the status of the pause
    return 0
#Gets the new feed 
def get_published():
    publisher_string = "f100s10r45b500d30f5"
    publisher_data = ["".join(x) for _, x in itertools.groupby(publisher_string, key=str.isdigit)]
    #Publisher Data is an array of alternating chars and integers
    return publisher_data

def main():
    ports = registerdevices()
    body = motor_control_class(ports['motor'])
    arm = arm_control_class(ports['arm'])
    repeat = True #Just for now
    while(repeat):
        path = get_published()
        body.sendstatus(1) #We are in the loop and busy
        ##The Loop
        #Only iterate through the odd numbers that should have a letter
        for x in [x for x in range(len(path)) if not(x % 2)]:
            #Exploits a dictionary to create a switch statement
            {
                'f': body.forward,
                'b': body.backward,
                's': body.setspeed,
                'r': body.rotate,
                'd': arm.base_rotate
            } [path[x]] (path[x+1])
            body.halt()
        ##
        repeat = False

main()
