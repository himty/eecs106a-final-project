#!/usr/bin/env python
import rospy
import cv2
import sys

from command_sphere_msg.msg import CommandSphere
from stamped_command_spheres_msg.msg import StampedCommandSpheres
from vision import CVSpheres

def runVision(filename):
	# Load data and calibrate
	cvs = CVSpheres.fromConfigFile(filename)
	# Set up publisher
	pub = rospy.Publisher('vision_spheres', StampedCommandSpheres, queue_size=10)

	while not rospy.is_shutdown():
		spheres = cvs.step()
		cmdSpheres = [CommandSphere(s.cmdName, s.x, s.y, s.z) for s in spheres]
		msg = StampedCommandSpheres(cmdSpheres, rospy.get_time())
		pub.publish(msg)
		if cv2.waitKey(1) & 0xFF == ord('q'):
			break

if __name__ == '__main__':
	if len(sys.argv) <= 1:
		print("Usage: visionNode.py <configFilePath>")
		sys.exit(1)

	rospy.init_node('vision_input', anonymous=True)
	try:
		runVision(sys.argv[1])
	except rospy.ROSInterruptException:
		pass
