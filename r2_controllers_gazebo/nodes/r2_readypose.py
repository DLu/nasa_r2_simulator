#!/usr/bin/env python  
import roslib;
roslib.load_manifest('r2_gazebo')
#roslib.load_manifest('r2_controllers')

import rospy
import tf

from std_msgs.msg import String
from sensor_msgs.msg import JointState

from r2_msgs.srv import *

import math
import time

def send_command_data() :

    print "setting left arm to joint mode"
    rospy.wait_for_service('r2_controller/set_joint_mode')
    try:
        set_joint_mode = rospy.ServiceProxy('r2_controller/set_joint_mode', SetJointMode)
        resp1 = set_joint_mode('left')
        #print resp1.result
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

    print "setting right arm to joint mode"
    rospy.wait_for_service('r2_controller/set_joint_mode')
    try:
        set_joint_mode = rospy.ServiceProxy('r2_controller/set_joint_mode', SetJointMode)
        resp1 = set_joint_mode('right')
        #print resp1.result
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

    # joint command publishers
    left_arm_pub = rospy.Publisher('r2_controller/left_arm/joint_command', JointState)
    right_arm_pub = rospy.Publisher('r2_controller/right_arm/joint_command', JointState)
    neck_pub = rospy.Publisher('r2_controller/neck/joint_command', JointState)
    waist_pub = rospy.Publisher('r2_controller/waist/joint_command', JointState)

    rospy.init_node('r2_readypose')

    TORAD = math.pi/180.0

    waist_js = JointState()
    left_arm_js = JointState()
    right_arm_js = JointState()
    neck_js = JointState()

    left_arm_js.name = ['left/j0', 
                        'left/j1', 
                        'left/j2', 
                        'left/j3', 
                        'left/j4', 
                        'left/j5', 
                        'left/j6',
			'left_thumb/j0', 
                        'left_thumb/j2', 
                        'left_thumb/j3', 
                        'left_thumb/j4', 
                        'left_index/j0', 
                        'left_index/j1', 
                        'left_index/j2', 
                        'left_middle/j0', 
                        'left_middle/j1', 
                        'left_middle/j2', 
                        'left_ring/j0', 
                        'left_little/j0'] 

    right_arm_js.name = ['right/j0', 
                         'right/j1', 
                         'right/j2', 
                         'right/j3',                    
                         'right/j4', 
                         'right/j5', 
                         'right/j6',
			 'right_thumb/j0', 
                         'right_thumb/j2', 
                         'right_thumb/j3', 
                         'right_thumb/j4', 
                         'right_index/j0', 
                         'right_index/j1', 
                         'right_index/j2', 
                         'right_middle/j0', 
                         'right_middle/j1', 
                         'right_middle/j2', 
                         'right_ring/j0', 
                         'right_little/j0']

    neck_js.name = ['neck/j0', 
                    'neck/j1', 
                    'neck/j2']
    waist_js.name = ['waist/j0']


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
    left_arm_pub.publish(left_arm_js); #rospy.sleep(1)
    right_arm_pub.publish(right_arm_js); #rospy.sleep(1)
    neck_pub.publish(neck_js); #rospy.sleep(0.2)
    
    rospy.sleep(5)
    waist_pub.publish(waist_js); #rospy.sleep(3)
   
    print "subscriber shutting down..."


if __name__ == '__main__':
    try:
        send_command_data()
    except rospy.ROSInterruptException: pass
