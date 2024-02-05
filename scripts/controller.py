#!/usr/bin/env python3

#imports ROS for developing the node
import rospy

from turtlesim.msg import Pose

import math

# imports geometry/msgs/Twist for control commands
from geometry_msgs.msg import Twist

from robotics_lab1.msg import Turtlecontrol

pos_msg = Pose()
ctrl_msg = Turtlecontrol()

def pose_callback(data):
# Defining a subscriber callback function
    global pos_msg
    pos_msg = data

def control_callback(data):
# Defining a subscriber callback function
    global ctrl_msg
    ctrl_msg = data

if __name__ == '__main__':
	#initializes the node
    rospy.init_node("turtle_controller", anonymous = True)
    # Adding subscribers to read the position information
    pose_subscriber = rospy.Subscriber('/turtle1/pose',Pose,pose_callback)
    control_subscriber = rospy.Subscriber('/turtle1/control_params',Turtlecontrol,control_callback)
    # Declares a publisher ub the velocity command topic
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel',Twist,queue_size=10)

    loop_rate = rospy.Rate(10)
    
    vel_msg = Twist()

    while not rospy.is_shutdown():
        vel_msg.linear.x = ctrl_msg.kp * (ctrl_msg.xd - pos_msg.x)
        print(ctrl_msg.xd)
        print(ctrl_msg.kp)
        print(ctrl_msg.xd - pos_msg.x)
        print("")
        velocity_publisher.publish(vel_msg)
        loop_rate.sleep()
