#!/usr/bin/python2
import socket

import rospy
from std_msgs.msg import String

def create_socket_server():
    HOST = ''# Symbolic name meaning all available interfaces
    PORT = 35801# Arbitrary non-privileged port
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind((HOST, PORT))
    s.listen(1)
    return s

def init_ros(publish, node_name):
    motor_pub = rospy.Publisher(publish, String)
    rospy.init_node(node_name)

if __name__ == "__main__":
    s = create_socket_server()
    motor_pub = rospy.Publisher("arm_commands", String)
    pub = init_ros("arm_command", "navigation")
    conn, addr = s.accept()
    print 'Connected by', addr
    while not rospy.is_shutdown():
        data = conn.recv(1024)
        if not data: continue
        #flush the queue we dont want this stuff building up
        motor_pub.publish(data)
        print data
        conn.sendall("r")
    conn.close()
