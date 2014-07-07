#!/usr/bin/env python  
import roslib;
roslib.load_manifest('r2_gazebo')
#roslib.load_manifest('r2_controllers')

import rospy
import tf

from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

from tf.transformations import quaternion_from_euler
from nasa_r2_common_msgs.srv import *

import math
import time

def send_command_data() :

    print "setting left arm to joint mode"
    rospy.wait_for_service('/r2_controller/set_joint_mode')
    try:
        set_joint_mode = rospy.ServiceProxy('/r2_controller/set_joint_mode', SetJointMode)
        resp1 = set_joint_mode('left')
        #print resp1.result
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

    print "setting right arm to joint mode"
    rospy.wait_for_service('/r2_controller/set_joint_mode')
    try:
        set_joint_mode = rospy.ServiceProxy('/r2_controller/set_joint_mode', SetJointMode)
        resp1 = set_joint_mode('right')
        #print resp1.result
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

    # joint command publishers
    left_arm_pub = rospy.Publisher('/r2_controller/left_arm/joint_command', JointState)
    right_arm_pub = rospy.Publisher('/r2_controller/right_arm/joint_command', JointState)
    neck_pub = rospy.Publisher('/r2_controller/neck/joint_command', JointState)
    waist_pub = rospy.Publisher('/r2_controller/waist/joint_command', JointState)

    # pose command publishers
    left_pose_pub = rospy.Publisher('/r2_controller/left/pose_command', PoseStamped)
    right_pose_pub = rospy.Publisher('/r2_controller/right/pose_command', PoseStamped)

    rospy.init_node('r2_gazebo_controller_test')

    TORAD = math.pi/180.0

    waist_js = JointState()
    left_arm_js = JointState()
    right_arm_js = JointState()
    neck_js = JointState()

    left_pose = PoseStamped()
    right_pose = PoseStamped()

    left_arm_js.name = ['/r2/left_arm/joint0', 
                        '/r2/left_arm/joint1', 
                        '/r2/left_arm/joint2', 
                        '/r2/left_arm/joint3', 
                        '/r2/left_arm/joint4', 
                        '/r2/left_arm/joint5', 
                        '/r2/left_arm/joint6',
			'/r2/left_arm/hand/thumb/joint0', 
                        '/r2/left_arm/hand/thumb/joint1', 
                        '/r2/left_arm/hand/thumb/joint2', 
                        '/r2/left_arm/hand/thumb/joint3', 
                        '/r2/left_arm/hand/index/joint0', 
                        '/r2/left_arm/hand/index/joint1', 
                        '/r2/left_arm/hand/index/joint2', 
                        '/r2/left_arm/hand/middle/joint0', 
                        '/r2/left_arm/hand/middle/joint1', 
                        '/r2/left_arm/hand/middle/joint2', 
                        '/r2/left_arm/hand/ring/joint0', 
                        '/r2/left_arm/hand/little/joint0'] 

    right_arm_js.name = ['/r2/right_arm/joint0', 
                         '/r2/right_arm/joint1', 
                         '/r2/right_arm/joint2', 
                         '/r2/right_arm/joint3',                    
                         '/r2/right_arm/joint4', 
                         '/r2/right_arm/joint5', 
                         '/r2/right_arm/joint6',
			 '/r2/right_arm/hand/thumb/joint0', 
                         '/r2/right_arm/hand/thumb/joint1', 
                         '/r2/right_arm/hand/thumb/joint2', 
                         '/r2/right_arm/hand/thumb/joint3', 
                         '/r2/right_arm/hand/index/joint0', 
                         '/r2/right_arm/hand/index/joint1', 
                         '/r2/right_arm/hand/index/joint2', 
                         '/r2/right_arm/hand/middle/joint0', 
                         '/r2/right_arm/hand/middle/joint1', 
                         '/r2/right_arm/hand/middle/joint2', 
                         '/r2/right_arm/hand/ring/joint0', 
                         '/r2/right_arm/hand/little/joint0']

    neck_js.name = ['/r2/neck/joint0', 
                    '/r2/neck/joint1', 
                    '/r2/neck/joint2']
    waist_js.name = ['/r2/waist/joint0']


    left_arm_js.position = [0]*(7+12)
    right_arm_js.position = [0]*(7+12)
    neck_js.position = [0]*3
    waist_js.position = [180.0*TORAD]*1

    #left_arm_js.velocity = [0]*(7+12)
    #right_arm_js.velocity = [0]*(7+12)
    #neck_js.velocity = [0]*3
    #waist_js.velocity = [0]*1

    #left_arm_js.effort = [0]*(7+12)
    #right_arm_js.effort = [0]*(7+12)
    #neck_js.effort = [0]*3
    #waist_js.effort = [0]*1

    left_arm_js.header.seq = 0
    right_arm_js.header.seq = 0
    waist_js.header.seq = 0
    neck_js.header.seq = 0

    left_arm_js.header.stamp = rospy.get_rostime()
    right_arm_js.header.stamp = rospy.get_rostime()
    waist_js.header.stamp = rospy.get_rostime()
    neck_js.header.stamp = rospy.get_rostime()

    left_arm_js.header.frame_id = "world"
    right_arm_js.header.frame_id = "world"
    waist_js.header.frame_id = "world"
    neck_js.header.frame_id = "world"

    c = 0
    go_to_ready = 0

    left_arm_js.position = [50.0*TORAD, -80.0*TORAD, -105.0*TORAD, -140.0*TORAD, 80.0*TORAD, 0.0*TORAD, 0.0*TORAD]+[0*TORAD]*12
    right_arm_js.position = [-50.0*TORAD, -80.0*TORAD, 105.0*TORAD, -140.0*TORAD, -80.0*TORAD, 0.0*TORAD, 0.0*TORAD]+[0*TORAD]*12
    neck_js.position = [-5*TORAD, -0*TORAD, -0*TORAD]
    waist_js.position = [180.0*TORAD]*1

    rospy.sleep(1)        
    waist_pub.publish(waist_js); #rospy.sleep(3)
    left_arm_pub.publish(left_arm_js); #rospy.sleep(1)
    right_arm_pub.publish(right_arm_js); #rospy.sleep(1)
    neck_pub.publish(neck_js); #rospy.sleep(0.2)
    
    rospy.sleep(7)

    print "setting left tip to palm"
    rospy.wait_for_service('/r2_controller/set_tip_name')
    try:
        set_tip_name = rospy.ServiceProxy('/r2_controller/set_tip_name', SetTipName)
        resp1 = set_tip_name('left', '/r2/left_palm')
        print resp1.result
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

    print "setting right tip to palm"
    rospy.wait_for_service('/r2_controller/set_tip_name')
    try:
        set_tip_name = rospy.ServiceProxy('/r2_controller/set_tip_name', SetTipName)
        resp1 = set_tip_name('right', '/r2/right_palm')
        print resp1.result
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

    # create pose message
    # set orientations
    roll = -1.57
    pitch = 0
    yaw = 0
    q = quaternion_from_euler(roll, pitch, yaw)

    left_pose.header.seq = 0
    left_pose.header.stamp = rospy.get_rostime()
    left_pose.header.frame_id = '/r2/robot_reference'
    left_pose.pose.position.x = 0.6
    left_pose.pose.position.y = -.35
    left_pose.pose.position.z = -.5
    left_pose.pose.orientation.x = q[0]
    left_pose.pose.orientation.y = q[1]
    left_pose.pose.orientation.z = q[2]
    left_pose.pose.orientation.w = q[3]
    print left_pose.pose.orientation
    left_pose_pub.publish(left_pose); 

    roll = 0
    pitch = 0
    yaw = 0
    q = quaternion_from_euler(roll, pitch, yaw)

    right_pose.header.seq = 0
    right_pose.header.stamp = rospy.get_rostime()
    right_pose.header.frame_id = '/r2/robot_reference'
    right_pose.pose.position.x = 0.6
    right_pose.pose.position.y = .35
    right_pose.pose.position.z = -.5
    right_pose.pose.orientation.x = q[0]
    right_pose.pose.orientation.y = q[1]
    right_pose.pose.orientation.z = q[2]
    right_pose.pose.orientation.w = q[3]
    print right_pose.pose.orientation
    right_pose_pub.publish(right_pose); 

    rospy.sleep(2)

    while not rospy.is_shutdown():
            
	if c%2 == 0:
	    go_to_ready = 1-go_to_ready
	    print "going to ready: ", go_to_ready	

	if(go_to_ready): 
            left_pose.pose.position.x = 0.6
            left_pose.pose.position.y = -.5
            left_pose.pose.position.z = -.5

            right_pose.pose.position.x = 0.6
            right_pose.pose.position.y = .5
            right_pose.pose.position.z = -.5

            roll = -1.57
            pitch = 0
            yaw = 0

            q = quaternion_from_euler(roll, pitch, yaw)
            left_pose.pose.orientation.x = q[0]
            left_pose.pose.orientation.y = q[1]
            left_pose.pose.orientation.z = q[2]
            left_pose.pose.orientation.w = q[3]
            print left_pose.pose.orientation


	else :

            left_pose.pose.position.x = 0.6
            left_pose.pose.position.y = -.5
            left_pose.pose.position.z = -.7

            right_pose.pose.position.x = 0.6
            right_pose.pose.position.y = .5
            right_pose.pose.position.z = -.7

            roll = 0
            pitch = 0
            yaw = 0
            
            q = quaternion_from_euler(roll, pitch, yaw)
            left_pose.pose.orientation.x = q[0]
            left_pose.pose.orientation.y = q[1]
            left_pose.pose.orientation.z = q[2]
            left_pose.pose.orientation.w = q[3]
            print left_pose.pose.orientation

        waist_pub.publish(waist_js); #rospy.sleep(3)

        left_pose.header.seq = c
        left_pose.header.stamp = rospy.get_rostime()
        left_pose_pub.publish(left_pose); #rospy.sleep(0.2)

        right_pose.header.seq = c
        right_pose.header.stamp = rospy.get_rostime()
        right_pose_pub.publish(right_pose); #rospy.sleep(0.2)

        rospy.sleep(2)
 	c += 1

    print "subscriber shutting down..."


if __name__ == '__main__':
    try:
        send_command_data()
    except rospy.ROSInterruptException: pass
