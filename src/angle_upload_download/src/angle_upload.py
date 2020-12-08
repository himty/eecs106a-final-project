#!/usr/bin/env python

"""
ROS node for upload angle to remote server.
"""

import rospy
from AngleArr.msg import AngleArr
from time import ctime
import socket

host='66.42.78.120'
port=812
addr=(host, port)
bufsize=1024

def callback(message):

    # callback program
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect(addr)
    data = str(message.angles[0]) + ' ' + str(message.angles[1]) + ' ' + str(message.angles[1]) + ' ' + str(message.timestamp)# encode as string
    if not data:
        return
    sock.send(data.encode(encoding='utf_8', errors='strict'))
    
    


def angle_upload():

    #Create a new instance of the rospy.Subscriber object which we can 
    #use to receive messages of type std_msgs/String from the topic /chatter_talk.
    #Whenever a new message is received, the method callback() will be called
    #with the received message as its first argument.
    rospy.Subscriber("cmd_angle", AngleArr, callback)


    #Wait for messages to arrive on the subscribed topics, and exit the node
    #when it is killed with Ctrl+C
    rospy.spin()

#Python's syntax for a main() method
if __name__ == '__main__':

	rospy.init_node('angle_upload', anonymous=True)
	  
	try:
	  angle_upload()

	except rospy.ROSInterruptException: pass