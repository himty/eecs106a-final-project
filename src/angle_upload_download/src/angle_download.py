#!/usr/bin/env python

"""
ROS node for upload angle to remote server.
"""

import rospy
from AngleArr.msg import AngleArr
from time import ctime
import socket

host='66.42.78.120'
port=813
addr=(host, port)
bufsize=1024


def angle_download():

  
  pub = rospy.Publisher('cmd_angle', AngleArr, queue_size=10)

  
  # Create a timer object that will sleep long enough to result in
  # a 100Hz publishing rate
  r = rospy.Rate(100) # 100hz

  # Loop until the node is killed with Ctrl-C
  while not rospy.is_shutdown():
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect(addr)
    data = 'angle_request'
    sock.send(data.encode(encoding='utf_8', errors='strict'))
    data = sock.recv(bufsize)
    tdata = data.decode(encoding='utf_8', errors='strict')
    tdata = tdata.split(' ')
    angles = [int(tdata[0]), int(tdata[1]), int(tdata[2])]
    pub_string = AngleArr(angles, float(tdata[3]))
    
    # Publish our string to the 'cmd_angle' topic
    pub.publish(pub_string)
    
    # Use our rate object to sleep until it is time to publish again
    r.sleep()


#Python's syntax for a main() method
if __name__ == '__main__':

	rospy.init_node('angle_download', anonymous=True)
	  
	try:
	  angle_download()

	except rospy.ROSInterruptException: pass