#! /usr/bin/env python
import rospy
import time
import numpy as np
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2
from math import sqrt
 
x = -1.5
y = -1.75
theta = 0.0 

def newOdom(msg):
    global x
    global y
    global theta
    
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
    
rospy.init_node("speed_controller")

sub = rospy.Subscriber("/odom", Odometry, newOdom)
pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)
 
speed = Twist()
 
r = rospy.Rate(10)

while not rospy.is_shutdown():
    print('theta:',theta)
    
