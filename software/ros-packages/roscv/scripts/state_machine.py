import rospy
from std_msgs.msg import String


class StateMachine(object):

    def __init__(self, state_machine):
        self.state = "FindingObject"
        self.prevState = None
        rospy.Subscriber("Object", String, self.Object_Message)
        rospy.Subscriber("Obstacle", String, self.Obstacle_Message)
        rospy.Subscriber("Process_Management", String, self.Processes)

    def Object_Message(self, data):
        if data == "Object Found":
            if self.State == "FindingObject":
               self.prevState = self.State
               self.State = "MoveTowardObject"

        elif data == "At Object":
            self.prevState = self.State
            self.State = "Pickup Object"

        elif data == "Object Picked up":
            self.prevState = self.State
            self.State = "Find Base Station"

        else:
            print "Error"

    def Obstacle_Message(self, data):
        if data == "Blocked":
            if self.State == "Pickup Object":
                pass
            else:
                self.prevState = self.State
                self.State == "Avoid Obstacle"
        elif data == "Not Blocked":
            if self.State == "Avoid Obstacle":
                self.State = self.prevState
        else:
            print "Error"

    def Processes(self, data):
        print "Process"



if __name__ == '__main__':
    try:
        rospy.init_node("state_machine")
        smachine = StateMachine("state_machine")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
