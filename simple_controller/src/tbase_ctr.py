#! /usr/bin/env python 

import rospy
import time
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2

#beginning coord

x = 0.0
y = 0.0
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
 
r = rospy.Rate(100)
 
nodes_ang = [-2, 0.75061866 , -0.1059124  , 0.62414946 , -0.62414946 ,
  1.06112263 , -1.06112263 , -1.01879139]
nodes_dis = [ 4.527692569068709, 12.419742348374221, 8.514693182963201, 4.6097722286464435, 4.6097722286464435, 4.031128874149275, 4.6097722286464435, 8.0156097709407]
  
#desired location
i = 0
j=0
start_time = time.time()
ang_time = abs(nodes_ang[j])
ang_sign = nodes_ang[j]
dis_time = nodes_dis[j]
start_time2 = 0
 
while not rospy.is_shutdown():
   
   while (time.time() - start_time < ang_time):
        if (ang_sign < 0):
            speed.linear.x = 0.0
            speed.angular.z = 0.7854
        else:
            speed.linear.x = 0.0
            speed.angular.z = -0.7854
        print(time.time() - start_time)
        pub.publish(speed)
   
   start_time2 = time.time()
   print(start_time2)
   while (time.time() - start_time2 < dis_time):
         speed.linear.x = 0.1
         speed.angular.z = 0.0
         print('Straight')
         pub.publish(speed)
   speed.linear.x = 0.0
   speed.angular.z= 0.0 
   pub.publish(speed)
   print('Next')
   j+=30
   ang_time = abs(nodes_ang[j])
   ang_sign = nodes_ang[j]
   dis_time = nodes_dis[j]
   start_time=time.time()
   r.sleep()
 
 
