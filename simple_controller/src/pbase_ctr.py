#! /usr/bin/env python 

import rospy
import time
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2

#beginning coord

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
pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
 
speed = Twist()
 
r = rospy.Rate(4)
 
nodes_x = [-.5]
nodes_y = [-1.75]
 
#desired location
i = 0
goal = Point()
goal.x = nodes_x[i]
goal.y = nodes_y[i]
 
while not rospy.is_shutdown():
   inc_x = goal.x - x
   inc_y = goal.y - y
   
    
   angle_to_goal = atan2(inc_y, inc_x)
    
   if abs(angle_to_goal - theta) > 0.1:
       speed.linear.x = 0.0
       speed.angular.z = 0.3
   elif abs(inc_x) < 0.1 and abs(inc_y) < 0.1:
       i+= 1
       goal.x = nodes_x[i]
       goal.y = nodes_y[i]  
   else:
       speed.linear.x = 0.5
       speed.angular.z = 0.0
           
  
   pub.publish(speed)
   r.sleep()
 
 
