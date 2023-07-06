#! /usr/bin/env python 

#This code uses the time values outputed by the path planning code to ...
#tell the robot how long to move foward and how long to turn for.
#It makes sure to adjust for distance and angle errors. It has an adapted gain
#which increases speed based of if the robot did not reach the node at the desired time.
#At the end it prints out the total time it took and the total predicted time.    
import rospy
import time
import numpy as np
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2
from math import sqrt

#beginning coord is introduced (with respect to the path planning code)

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
 
#import time values from path planning program
r = rospy.Rate(100)
nodes_dis = [0.0, 4.5, 4.6097722286464435, 4.527692569068709, 4.5, 4.527692569068709, 8.077747210701755, 4.6097722286464435, 4.6097722286464435, 4.6097722286464435, 7.5]
nodes_ang = [0.0, 1.7224552623196874, 1.468244582440341, 0.7115914845140923, 0.7115914845140923, 1.7224552623196874, 1.7224552623196874, 2.0010144290380847, 0.702745200673308, -0.14096461296037663, -0.14096461296037663, -0.09779858760481279]


#adjusting times for velocity that is used by turtlebot
time_ang = (np.array(nodes_ang)*(-0.785))
time_ang = abs(np.array(time_ang))
time_ang = np.array(time_ang)*(1/0.1)

tot_lin_time = []
tot_ang_time = []
#calculating how much time required in traveling foward and turning
tot_time_dis = np.sum(nodes_dis)
tot_time_ang = np.sum(time_ang)
i = 0
j = 0

#timer implementation
start_time = time.time()
start_time2 = 0
start_time3 = 0

#calculating how much turtlebot should turn by from current orientation
angle2goal= (nodes_ang[j]*-.785) + (np.pi/2)
ang_sign = nodes_ang[j]
dis_time = nodes_dis[j] 

#setting up new variables for distance calculation
x1 = x
y1 = y
x2 = 0
y2 = 0
gain = 0
angstart_time = 0

run_time = time.time() 
while not rospy.is_shutdown():
   
   #checks if pos and tells it to move foward until it meets a 
   #small difference of angles
   
   
   if (angle2goal - theta) > 0: 
    angstart_time = time.time() 
    while abs(angle2goal - theta) > 0.01:
         speed.linear.x = 0.0
         speed.angular.z = 0.105
         pub.publish(speed)    
   else:
    #turn CW if not pos
    angstart_time = time.time() 
    while abs(angle2goal - theta) > 0.01:     
         speed.linear.x = 0.0
         speed.angular.z = -0.105
         pub.publish(speed)
   ang_time = time.time() - angstart_time     
        # print(angle2goal - theta)     
   
   r.sleep()
   b = tot_ang_time
   b.append(ang_time)
   print('test', ang_time)
   print('real', time_ang[j])
   start_time2 = time.time() 
   #print('Current angle',theta)
   #print('angle2goal', angle2goal)
   #print('angle error',angle2goal - theta)
   speed.linear.x = 0.0
   speed.angular.z = 0.0
   pub.publish(speed)
   
   #makes sure robot travels the correct amount of time that is outputed by path planning code
   linstart_time = time.time()
   while (time.time() - start_time2 < dis_time):
         speed.linear.x = 0.10
         speed.angular.z = 0.0   
         pub.publish(speed)
         
   speed.linear.x = 0.0
   speed.angular.z = 0.0  
   pub.publish(speed)
   r.sleep() 
   
         
   dist_needed = abs(nodes_dis[j])*(0.1)
   x2 = x
   y2 = y
   dist_trav = sqrt((x2 - x1)**2 + (y2 - y1)**2)
   #calculates and compares actual distance to distance traveled
   #print('Distance needed', dist_needed)
   #print('Distance traveled', dist_trav)
   dist_diff  = abs(dist_needed - dist_trav)
   time_diff = dist_diff * (1/0.1)
   start_time3 = time.time()
   #print('distance error', dist_diff)
   
   #corrects if error is too large
   while abs(time.time() - start_time3 < time_diff):
         speed.linear.x = 0.10
         speed.angular.z = 0.0 
         #print(time.time() - start_time3 < time_diff)
         pub.publish(speed)
         
   speed.linear.x = 0.0
   speed.angular.z = 0.0
   pub.publish(speed)
   
   #keeps track of linear_time travled
   linear_time = time.time() - linstart_time
   a = tot_lin_time
   a.append(linear_time)
   r.sleep()
   #calculates a time ratio to determine if robot should speed up to meet the time goal
   time_ratio = nodes_dis[j]/linear_time
   print(time_ratio)
   x2 = x
   y2 = y
   dist_diff  = abs(dist_needed - dist_trav)
   #print('distance error corrected', dist_diff)
   x1 = x2
   y1 = y2
   
   print('node:',j+1)
   j+=1
   if (time_ratio < .98):
        gain =  gain + 0.01
        print('GAIN INCREASED')
        
   if (j > (len(nodes_dis)-1)):
        run_time = time.time() - run_time
        tot_act_lin = np.sum(tot_lin_time) 
        tot_act_ang = np.sum(tot_ang_time)
        print(tot_act_lin + tot_act_ang)
        print(tot_time_ang)
        print(tot_act_ang)
        print(tot_time_dis)
        print(tot_act_lin)
        speed.linear.x = 0.0
        speed.angular.z = 0.0  
        pub.publish(speed)
        r.sleep()
 
   angle2goal= (nodes_ang[j]*-.785) + (np.pi/2)
   dis_time = nodes_dis[j] 
   start_time = time.time()
