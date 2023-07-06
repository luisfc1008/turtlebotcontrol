#! /usr/bin/env python 

import rospy
import time
import numpy as np
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2
from math import sqrt

#beginning coord

x = -1.5
y = -1.75
theta = 0.0 

class Turtlebot():
    def __init__(self):
        rospy.init_node("turtlebot_move")
        rospy.loginfo("Press Ctrl + C to terminate")
        self.vel_pub = rospy.Publisher("cmd_vel_mux/input/navi", Twist, queue_size=10)
        self.rate = rospy.Rate(10)

        # reset odometry to zero
        self.reset_pub = rospy.Publisher("mobile_base/commands/reset_odometry", Empty, queue_size=10)
        for i in range(10):
            self.reset_pub.publish(Empty())
            self.rate.sleep()


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
nodes_ang = [1.46824458 , 1.25196653 , 0.90270863 , 1.41038053 , 0.7490479 ,
  0.71159148 , 0.14096461 , -0.27855917-.1 , 1.32365893]
nodes_dis = [4.924428900898052, 3.605551275463989, 4.6097722286464435, 4.47213595499958, 9.013878188659973, 4.716990566028302, 4.527692569068709, 4.6097722286464435, 9.86154146165801]
  
#desired location
i = 0
j=0
start_time = time.time()
start_time2 = 0
start_time3 = 0
angle2goal= (nodes_ang[j]*-.785) + (np.pi/2)
ang_sign = nodes_ang[j]
dis_time = nodes_dis[j] 
x1 = x
y1 = y
x2 = 0
y2 = 0
print(np.array(nodes_ang)*-.785 + (np.pi/2))
 
while not rospy.is_shutdown():
   
   
   while abs(angle2goal - theta) > 0.01:
         speed.linear.x = 0.0
         speed.angular.z = 0.1
         pub.publish(speed)
         r.sleep()
        # print(angle2goal - theta)     
   start_time2 = time.time() 
   print('Current angle',theta)
   print('angle2goal', angle2goal)
   print('angle error',angle2goal - theta)
   speed.linear.x = 0.0
   speed.angular.z = 0.0
   pub.publish(speed)
   while (time.time() - start_time2 < dis_time):
         speed.linear.x = 0.1
         speed.angular.z = 0.0   
         pub.publish(speed)
         r.sleep()
   speed.linear.x = 0.0
   speed.angular.z = 0.0
   pub.publish(speed)      
   dist_needed = abs(nodes_dis[j])*(0.1)
   x2 = x
   y2 = y
   dist_trav = sqrt((x2 - x1)**2 + (y2 - y1)**2)
   print('Distance needed', dist_needed)
   print('Distance traveled', dist_trav)
   x1 = x2
   y1 = y2
   dist_diff  = abs(dist_needed - dist_trav)
   time_diff = dist_diff * (1/0.1)
   start_time3 = time.time()
   print('distance error', dist_diff)
   
   while abs(time.time() - start_time3 < time_diff):
         speed.linear.x = 0.1
         speed.angular.z = 0.0 
         #print(time.time() - start_time3 < time_diff)
         pub.publish(speed)
         r.sleep()
   speed.linear.x = 0.0
   speed.angular.z = 0.0
   pub.publish(speed)
   r.sleep()
   print('node:',j+1)
   j+=1
   angle2goal= (nodes_ang[j]*-.785) + (np.pi/2)
   dis_time = nodes_dis[j] 
   start_time=time.time()
   if (j == (len(nodes_dis)-1)):
        speed.linear.x = 0.0
        speed.angular.z = 0.0  
   pub.publish(speed)
   r.sleep()
 
 
