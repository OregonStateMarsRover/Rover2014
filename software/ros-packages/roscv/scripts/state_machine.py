import rospy
from std_msgs.msg import String
import threading


class StateMachine(object):

    def __init__(self, state_machine):
        self.state = "SearchPattern"
        self.prevState = None
        self.objectPickedup = False
        rospy.Subscriber("state_change_request", String, self.state_callback)
        rospy.Subscriber("/motor_command/path_finding/", String, self.motor_callback)
        rospy.Subscriber("/motor_command/find_base_station/", String, self.motor_callback)
        rospy.Subscriber("/motor_command/object_detection/", String, self.motor_callback)
        self.motor_pub = rospy.Publisher("motor_command", String)
        self.state_pub = rospy.Publisher("state", String)
        self.state_thread = threading.Thread(target = self.print_state)

        self.state_thread.start()

    def print_state(self):
        print "Thread started"
        while True:
            self.state_pub.publish(self.state)
            rospy.sleep(.25)

    def motor_callback(self, data):
        topic = data._connection_header['topic']
        topicStates = {'/motor_command/path_finding/' : ['AvoidObstacle','SearchPattern'], '/motor_command/find_base_station/': ['FindBaseStation','FindBaseStationFinal'], '/motor_command/object_detection' : ['MoveTowardObject'] }
        for state in topicStates[topic]:
            if state == self.state:
                print state, topic, data.data
                self.motor_pub.publish(data.data)


    def state_callback(self, data):
        if self.state == "SearchPattern":
            if data.data == "Object Found" and not(self.objectPickedup):
                   print "Changing from FindingObject to MoveTowardObject"
                   self.prevState = self.state
                   self.state = "MoveTowardObject"

            elif data.data == "Object Found" and self.objectPickedup:
                print "Object already picked up, ignoring state switch request"

            elif data.data == "Base Station Found" and not(self.objectPickedup):
                print "Object not picked up, ignoring request to change state to FindBaseStation"

            elif data.data == "Base Station Found" and self.objectPickedup:
                print "Switching to FindBaseStation state"
                self.prevState = self.state
                self.State = "FindBaseStation"

            elif data.data == "Base Station Found Final" and not(self.objectPickedup):
                print "Object not picked up, ignoring request to change state to FindBaseStationFinal"

            elif data.data == "Base Station Found Final" and self.objectPickedup:
                print "Switching to FindBasestationFinal state"
                self.prevState = self.state
                self.State = "FindBaseStationFinal"

            elif data.data == "Blocked":
                print "Switching to AvoidObstacle state"
                self.prevState = self.state
                self.state = "AvoidObstacle"

            else:
                print "Ignoring state change request %s becuase in state SearchPattern" % data.data

        elif self.state == "MoveTowardObject":
            if data.data == "At Object":
                print "Changing from MoveTowardObject to PickupObject state"
                self.prevState = self.state
                self.state = "PickupObject"

            elif data.data == "Blocked":
                print "Changing frm MoveTowardObject to AvoidObstacle state"
                self.prevState = self.state
                self.state = "AvoidObstacle"

            else:
                print "Ignoring state change request %s becuase in state MoveTowardObject" % data.data

        elif self.state == "AvoidObstacle":
            if data.data == "Not Blocked":
                print "Changing from  AvoidObstacle to prevstate: %s" % self.prevState
                self.state = self.prevState
                self.prevState = "AvoidObstacle"

            else:
                print "Ignoring state change request %s becuase in state AvoidObstacle state" % data.data

        elif self.state == "FindBaseStation":
            if data.data == "Blocked":
                print "Changing from FindBaseStation to AvoidObstacle state"
                self.prevState = self.state
                self.state = "AvoidObstacle"

            else:
                print "Ignoring state change request %s becuase in state FindBaseStation state" % data.data

        elif self.state == "FindBaseStationFinal":
            if data.data == "At Base Station":
                print "Changing from FindBaseStationFinal to Final state"
                self.prevState = self.state
                self.state = "Final"

            else:
                print "Ignoring state change request %s becuase in state FindBaseStationFinal state" % data.data

        elif self.state == "PickupObject":
                if data.data == "Object Retrieved":
                    print "Object retrieved"
                    print "Changing from PickupObject to SearchPattern"
                    self.prevState = self.state
                    self.objectPickedup = True
                    self.state = "SearchPattern"

                else:
                    print "Ignoring state change request %s becuase in state PickupObject state" % data.data

        #TODO: Fix this to work with process manager
        elif self.state == "ProcessManaging":
            print "In ProcessManaging state, ignoring"

        else:
            print "In unknown state: Error"





if __name__ == '__main__':
    try:
        rospy.init_node("state_machine")
        smachine = StateMachine("state_machine")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
