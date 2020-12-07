#!/usr/bin/env python

"""
ROS network for path planning and output
"""

import rospy
#import pathplanning package
#import CV package

from robot_comm_msg.msg import AngleArr

def cmd_angle():

    pub = rospy.Publisher('cmd_angle', AngleArr, queue_size=10)
    
    r = rospy.Rate(10) # 10hz


    while not rospy.is_shutdown():

        raw_input('Press enter to actuate arm:')

        try:
            target_coords = cv()

			angles = plan_path(target_coords) #uint16[] of joint angles
            
            print(angles)

            pub_string = AngleArr(angles, rospy.get_time())

        	pub.publish(pub_string)

        	r.sleep()
            
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

#Python's syntax for a main() method
if __name__ == '__main__':

	rospy.init_node('cmd_angle', anonymous=True)
	  
	try:
	  cmd_angle()

	except rospy.ROSInterruptException: pass