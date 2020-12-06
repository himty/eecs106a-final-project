#!/usr/bin/env python
import rospy
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped

def main():
    #Wait for the IK service to become available
    rospy.wait_for_service('compute_ik')
    rospy.init_node('service_query')
    
    #Create the function used to call the service
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
    
    while not rospy.is_shutdown():
        raw_input('Press enter to compute an IK solution:')
        
        #Construct the request
        request = GetPositionIKRequest()
        request.ik_request.group_name = "arm"
        request.ik_request.ik_link_name = "link3"
        request.ik_request.attempts = 20
        request.ik_request.pose_stamped.header.frame_id = "base1"
        
        #YOUR CODE HERE
        #Write code to get the (x, y, z) coordinates of the end effector from the user
        #Hint: the function raw_input() might be useful
        x = float(raw_input('x coords of end effector: '))
        y = float(raw_input('y coords of end effector: '))
        z = float(raw_input('z coords of end effector: '))
        
        #Set the desired orientation for the end effector HERE
        request.ik_request.pose_stamped.pose.position.x = x
        request.ik_request.pose_stamped.pose.position.y = y
        request.ik_request.pose_stamped.pose.position.z = z
        
        request.ik_request.pose_stamped.pose.orientation.x = 0.0
        request.ik_request.pose_stamped.pose.orientation.y = 1.0
        request.ik_request.pose_stamped.pose.orientation.z = 0.0
        request.ik_request.pose_stamped.pose.orientation.w = 0.0
        
        try:
            #Send the request to the service
            response = compute_ik(request)
            
            #Print the response HERE
            print(response)
            
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

#Python's syntax for a main() method
if __name__ == '__main__':
    main()