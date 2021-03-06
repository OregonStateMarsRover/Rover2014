#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import threading


class StateMachine(object):

    def __init__(self, state_machine):
        self.state = "SearchPattern"
        self.prevState = None
        self.objectPickedup = False
        rospy.Subscriber("state_change_request", String, self.state_callback)
        rospy.Subscriber("/motor_command/path_finding/", String, self.path_motor_callback)
        rospy.Subscriber("/motor_command/find_base_station/", String, self.base_motor_callback)
        rospy.Subscriber("/motor_command/object_detection/", String, self.object_motor_callback)
        rospy.Subscriber("/motor_command/search_pattern/", String, self.search_motor_callback)
        self.motor_pub = rospy.Publisher("motor_command", String)
        self.state_pub = rospy.Publisher("state", String)
        self.state_thread = threading.Thread(target = self.print_state)
        self.state_thread.daemon = True

        self.state_thread.start()

    def print_state(self):
        print "Thread started"
        while True:
            self.state_pub.publish(self.state)
            rospy.sleep(.25)
    def path_motor_callback(self,data):
        if self.state == "AvoidObstacle":
            self.motor_pub.publish(data.data)

    def object_motor_callback(self, data):
        if self.state == "MoveTowardObject" or self.state == "PickingupObject":
            self.motor_pub.publish(data.data)

    def search_motor_callback(self, data):
        if self.state == "SearchPattern" or self.state == "FindHome":
            self.motor_pub.publish(data.data)

    def base_motor_callback(self, data):
        if self.state == "FindBaseStation" or self.state == "FindBaseStationFinal":
            self.motor_pub.publish(data.data)
    
    def motor_callback(self, data):
        if 'topic' in data._connection_header:
            topic = data._connection_header['topic']
        else:
            if '/path_finding' in data._connection_header['callerid']:
                topic = '/motor_command/path_finding'
            elif '/search_pattern' in data._connection_header['callerid']:
                topic = '/motor_command/search_pattern'
            elif '/path_finding' in data._connection_header['callerid']:
                topic = '/motor_command/search_pattern'
            else:
                print "ERROR:::", data._connection_header['callerid']

        topicStates = {'/motor_command/path_finding' : ['SearchPattern'], '/motor_command/find_base_station': ['FindBaseStation','FindBaseStationFinal'], '/motor_command/object_detection' : ['MoveTowardObject'] , '/motor_command/search_pattern' : ['SearchPattern', 'FindHome']}
        print "topic is: ", topic
        print "topic states: ", topicStates
        for state in topicStates[topic]:
            print "state = ", state, state == self.state
            print "self.state = ", self.state,
            if state == self.state:
                print state, topic, data.data
                self.motor_pub.publish(data.data)


    def state_callback(self, data):
        if self.state == "SearchPattern":
            if data.data == "Object Found" and not(self.objectPickedup):
                   print "Changing from FindingObject to MoveTowardObject"
                   self.prevState = self.state
                   self.state = "MoveTowardObject"
                   self.motor_pub.publish("flush")

            elif data.data == "Object Found" and self.objectPickedup:
                print "Object already picked up, ignoring state switch request"

            elif data.data == "Base Station Found" and not(self.objectPickedup):
                print "Object not picked up, ignoring request to change state to FindBaseStation"

            elif data.data == "Base Station Found" and self.objectPickedup:
                print "Switching to FindBaseStation state"
                self.motor_pub.publish("flush")
                self.prevState = self.state
                self.state = "FindBaseStation"

            elif data.data == "Base Station Found Final" and not(self.objectPickedup):
                print "Object not picked up, ignoring request to change state to FindBaseStationFinal"

            elif data.data == "Base Station Found Final" and self.objectPickedup:
                print "Switching to FindBaseStationFinal state"
                self.prevState = self.state
                self.state = "FindBaseStationFinal"

            elif data.data == "Blocked":
                print "Switching to AvoidObstacle state"
                self.motor_pub.publish("flush")
                self.prevState = self.state
                self.state = "AvoidObstacle"

            else:
                print "Ignoring state change request %s becuase in state SearchPattern" % data.data

        elif self.state == "MoveTowardObject":
            if data.data == "At Object":
                print "Changing from MoveTowardObject to PickupObject state"
                self.prevState = self.state
                self.state = "PickupObject"
                self.motor_pub.publish("flush")
            elif data.data == "Object Lost":
                print "Changing from MoveTowardObject to SearchPattern state"
                self.prevState = self.state
                self.state = "SearchPattern"
                self.motor_pub.publish("flush")
                self.motor_pub.publish("rover")

            else:
                print "Ignoring state change request %s becuase in state MoveTowardObject" % data.data

        elif self.state == "FindHome":
            if data.data == "Object Found" and self.objectPickedup:
                print "Object already picked up, ignoring state switch request"

            elif data.data == "Base Station Found" and not(self.objectPickedup):
                print "Object not picked up, ignoring request to change state to FindBaseStation"

            elif data.data == "Base Station Found" and self.objectPickedup:
                print "Switching to FindBaseStation state"
                self.motor_pub.publish("flush")
                self.prevState = self.state
                self.state = "FindBaseStation"

            elif data.data == "Base Station Found Final" and not(self.objectPickedup):
                print "Object not picked up, ignoring request to change state to FindBaseStationFinal"

            elif data.data == "Base Station Found Final" and self.objectPickedup:
                print "Switching to FindBaseStationFinal state"
                self.prevState = self.state
                self.state = "FindBaseStationFinal"

            elif data.data == "Blocked":
                print "Switching to AvoidObstacle state"
                self.motor_pub.publish("flush")
                self.prevState = self.state
                self.state = "AvoidObstacle"

            else:
                print "Ignoring state change request %s becuase in state SearchPattern" % data.data

        elif self.state == "AvoidObstacle":
            if data.data == "Not Blocked":
                print "Changing from  AvoidObstacle to prevstate: %s" % self.prevState
                self.motor_pub.publish("flush")
                self.state = self.prevState
                self.prevState = "AvoidObstacle"

            else:
                print "Ignoring state change request %s becuase in state AvoidObstacle state" % data.data

        elif self.state == "FindBaseStation":
            if data.data == "Blocked":
                print "Changing from FindBaseStation to AvoidObstacle state"
                self.motor_pub.publish("flush")
                self.prevState = self.state
                self.state = "AvoidObstacle"

            elif data.data == "Base Station Found Final" and not(self.objectPickedup):
                print "Object not picked up, ignoring request to change state to FindBaseStationFinal"

            elif data.data == "Base Station Found Final" and self.objectPickedup:
                print "Switching to FindBaseStationFinal state"
                self.prevState = self.state
                self.state = "FindBaseStationFinal"
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
                self.motor_pub.publish("flush")
                print "Changing from PickupObject to FindHome"
                self.prevState = self.state
                self.objectPickedup = True
                self.state = "FindHome"
                self.motor_pub.publish("rover")
    
            elif data.data == "Object Lost":
                print "Object Lost, switching to SearchPattern"
                self.motor_pub.publish("flush")
                self.motor_pub.publish("rover")
                self.prevState = self.state
                self.state = "SearchPattern" 
            else: print "Ignoring state change request %s becuase in state PickupObject state" % data.data

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
