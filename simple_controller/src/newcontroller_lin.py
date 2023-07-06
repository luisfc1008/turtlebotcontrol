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

#some stuff 
class Turtlebot():
    def __init__(self):
        rospy.init_node("turtlebot_move")
        rospy.loginfo("Press Ctrl + C to terminate")
        self.vel_pub = rospy.Publisher("cmd_vel_mux/input/navi", Twist, queue_size=10)
        self.rate = rospy.Rate(100)

        # reset odometry to zero
        self.reset_pub = rospy.Publisher("mobile_base/commands/reset_odometry", Empty, queue_size=100)
        for i in range(100):
            self.reset_pub.publish(Empty())
            self.rate.sleep()
            
#defining x,y,theta variables
def newOdom(msg):
    global x
    global y
    global theta
    
    #assigning variables to their corresponding pose message 
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])        

rospy.init_node("speed_controller")

#Subcribing to odometry to recieve data from turtlebot wheels
sub = rospy.Subscriber("/odom", Odometry, newOdom)
#Publishing speed values to control movement
pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 100)

speed = Twist()

r = rospy.Rate(100)
nodes_dis = [0.0, 4.5, 4.6097722286464435, 4.527692569068709, 4.5, 4.527692569068709, 8.077747210701755, 4.6097722286464435, 4.6097722286464435, 4.6097722286464435, 7.5]
nodes_ang = [0.0, 1.7224552623196874, 1.468244582440341, 0.7115914845140923, 0.7115914845140923, 1.7224552623196874, 1.7224552623196874, 2.0010144290380847, 0.702745200673308, -0.14096461296037663, -0.14096461296037663, -0.09779858760481279]
#index
i = 1
j = 1

#inputs
lin_speed = 0.1
ang_speed = 0.785
u = lin_speed
kp_lin = 1
kd_lin = 0.2
angle2goal = (nodes_ang[j]*-.785) + (np.pi/2)
dist2goal = (nodes_dis[j]*lin_speed)
#adjusting times for velocity that is used by turtlebot
time_ang = (np.array(nodes_ang)*(-ang_speed))
time_ang = abs(np.array(time_ang))
time_ang = np.array(time_ang)*(1/0.1)
x1 = x
y1 = y
x2 = 0
y2 = 0
act_dist = 0
#fileopen1 = 

while not rospy.is_shutdown():

#linear travel code  
   if (dist2goal - act_dist) > 0:
        #print(i)
        while abs(dist2goal - act_dist) > 0.01:
            angstart_time = time.time()
            x2 = x
            y2 = y
            act_dist = sqrt((x2 - x1)**2 + (y2 - y1)**2)
            e = lin_speed + dist2goal - act_dist
            e_dot = 0 - speed.linear.x
            u = kp_lin * e + kd_lin * e_dot
            speed.linear.x = u
            pub.publish(speed)
            r.sleep()
            print(u)
            print(dist2goal - act_dist)
          
        print('next')
        speed.linear.x = 0
        pub.publish(speed)
        j+=1
        dist2goal = (nodes_dis[j]*lin_speed)
        x1 = x2
        y1 = y2
        x2 = x
        y2 = y
        act_dist = sqrt((x2 - x1)**2 + (y2 - y1)**2) 
        print(dist2goal - act_dist)   
            
            
           
   
   
   
   
     





