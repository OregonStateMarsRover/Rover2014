import rospy
from std_msgs.msg import String


class StateMachine(object):
    
    def __init__(self, state_machine):
        self.state = "FindingObject"
        self.prevState = None
        rospy.Subscriber("Object", String, self.Object_Message)
        rospy.Subscriber("Obstacle", String, self.Obstacle_Message)
        rospy.Subscriber("Process_Management", String, self.Processes)
	rospy.Subscriber("/motor_command/path_finding/", String, self.motor_callback)
	rospy.Subscriber("/motor_command/find_base_station/", String, self.motor_callback)
	rospy.Subscriber("/motor_command/object_detection/", String, self.motor_callback)
   
 

    def motor_callback(self, data):
	topic = data._connection_header['topic']
	topicStates = {'/motor_command/path_finding/' : ['AvoidObstacle','SearchPattern'], '/motor_command/find_base_station/': ['FindBaseStation'], '/motor_command/object_detection' : ['MoveTowardObject'] }
	for state in topicStates[topic]:
	    if state == self.state:
		print state, topic, data.data
		#pub.Publish(data.data) 
    
     
    def Object_Message(self, data):
	if self.state == "FindingObject":
	    if data.data == "Object Found":
               print "Changing from FindingObject to MoveTowardObject"
               self.prevState = self.state
               self.state = "MoveTowardObject"
	    else:
		print "In FindingObject state, ignoring"

	elif self.state == "MoveTowardObject":
	    if data.data == "At Object":
		self.prevState = self.state
		self.state = "PickupObject"
	    else:
		print "In MoveTowardObject state, ignoring"

        elif self.state == "AvoidObstacle":
	    print "In AvoidObstacle state, ignoring" 
	
	elif self.state == "FindBaseStation":
	    print "In FindBaseStation state, ignoring" 	
	
	elif self.state == "PickupObject":
            if data.data == "Object Retrieved":
		self.prevState = self.state
		self.state = "FindBaseStation"

	elif self.state == "ProcessManaging":
	    print "In ProcessManaging state, ignoring"

        else:
            print "In unknown state: Error"

    def Obstacle_Message(self, data):
        if data.data == "Blocked":
            if self.state == "Pickup Object":
                pass
            else:
                self.prevState = self.state
                self.state == "Avoid Obstacle"
        elif data.data == "Not Blocked":
            if self.state == "Avoid Obstacle":
                self.state = self.prevState
	    else:
		print "Rover was not blocked, so ignoring"
        else:
            print "Obstacle said something other than Blocked or Not Blocked: Error"

    def Processes(self, data):
        print "Process"



if __name__ == '__main__':
    try:
        rospy.init_node("state_machine")
        smachine = StateMachine("state_machine")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
