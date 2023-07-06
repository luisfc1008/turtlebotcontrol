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
        self.vel_pub = rospy.Publisher("cmd_vel_mux/input/navi", Twist, queue_size=100)
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
nodes_dis = [0.0, 4.5, 4.6097722286464435, 4.527692569068709, 4.5, 4.527692569068709] #, 8.077747210701755, 4.6097722286464435, 4.6097722286464435, 4.6097722286464435, 7.5]
nodes_ang = [0.0, 1.7224552623196874, 1.468244582440341, 0.7115914845140923, 0.7115914845140923, 1.7224552623196874]#, 1.7224552623196874, 2.0010144290380847, 0.702745200673308, -0.14096461296037663, -0.14096461296037663, -0.09779858760481279]
#index
i = 1
j = 1

#inputs
lin_speed = 0.1
ang_speed = 0.785
u = ang_speed
v = lin_speed
kp_ang = 0.39
kd_ang = 0.009
kp_lin = 0.4
kd_lin = 0.001
angle2goal = (nodes_ang[j]*-ang_speed) + (np.pi/2)
dist2goal = (nodes_dis[j]*lin_speed)

x1 = x
y1 = y
x2 = 0
y2 = 0
act_dist = 0

#adjusting times for velocity that is used by turtlebot
time_ang = (np.array(nodes_ang)*((-ang_speed)/0.1))
time_ang = abs(np.array(time_ang))


req_ang_time = np.sum(time_ang)
req_lin_time = np.sum(nodes_dis)

tot_ang_time = []
tot_lin_time = []
ang_error = []
lin_error = []
e_1 = 0
e_1_l = 0
a = []#tot_ang_time
b = []#tot_lin_time
c = []#ang_error
d = []#lin_error

f = open("angtime.txt","w+")
g = open("angle.txt","w+")
h = open("distime.txt", "w+")
i = open("dis.txt","w+")


while not rospy.is_shutdown():

#angle direction code  
   if (angle2goal - (theta)) > 0:
        u = ang_speed       
        angstart_time = time.time()
        while abs(angle2goal - theta) > 0.01:    
            speed.angular.z = u
            pub.publish(speed)
            e = angle2goal - (theta)
            #e_dot = 0 - speed.angular.z
            
            u = kp_ang * e + kd_ang * ((e - e_1)/0.01) #added error diff and sample time
            e_1= e
            
            
            ang_time = time.time()
            curr_ang = theta
         
            #print('yer')
            #print(theta)
            
   else:
        while abs(angle2goal - (theta)) > 0.01:
            #angstart_time = time.time()
            speed.angular.z = -u
            pub.publish(speed)
            e = (theta) + angle2goal
            #e_dot = 0 - speed.angular.z
     
            u = kp_ang * e + kd_ang * ((e - e_1)/0.01)
            e_1= e
            #ang_time = time.time() - angstart_time
            #a.append(ang_time)
            c.append(u)
            ang_time = time.time()
            curr_ang = theta
            f.write(str(ang_time))
            f.write("\n")
            g.write(str(curr_ang))
            g.write("\n")
            #print('ner')
            #print(theta)
   
   f.write(str(ang_time))
   f.write("\n")
   g.write(str(curr_ang))
   g.write("\n")
   ang_node_time = time.time() - angstart_time
   a.append(ang_node_time)
   angle2goal= (nodes_ang[j]*-.785) + (np.pi/2)

   
   speed.angular.z = 0
   pub.publish(speed)
   
   
   if (dist2goal - act_dist) > 0:
        #print(i)
        v = lin_speed
        linstart_time = time.time()
        while abs(dist2goal - act_dist) > 0.01:
            
            speed.linear.x = v
            pub.publish(speed)
            x2 = x
            y2 = y
            act_dist = sqrt((x2 - x1)**2 + (y2 - y1)**2)
            e_l = lin_speed + dist2goal - act_dist
            #e_dot_l = 0 - speed.linear.x
            v = kp_lin * e_l + kd_lin * ((e - e_1_l)/0.01) #added error diff and time sample
            e_1_l = e_1
            
            lin_time = time.time()
            h.write(str(lin_time))
            h.write("\n")
            i.write(str(act_dist))
            i.write("\n")
            r.sleep()
           # print(u)
           # print(dist2goal - act_dist)
        
         
        lin_node_time = time.time() - linstart_time
        b.append(lin_node_time)
        #print('next')
        speed.linear.x = 0
        pub.publish(speed)
        e_1 = 0
        e_1_l = 0
        
        x1 = x2
        y1 = y2
        x2 = x
        y2 = y
        act_dist = sqrt((x2 - x1)**2 + (y2 - y1)**2) 
        #print(dist2goal - act_dist)   
   
   
   j+=1
   if (j > (len(nodes_dis)-1)):    
        
        tot_act_ang = np.sum(a)
        tot_act_lin = np.sum(b) 
        
        print("Actual Ang time: ", tot_act_ang)
        print("req ang time: ", req_ang_time)
    
        print("Actual linear time: ", tot_act_lin)
        print("req lin time: ", req_lin_time)
        
        print("Total Act time: ", tot_act_lin + tot_act_ang)
        print("total req time: ", req_ang_time + req_lin_time)
         
        f.close()
        g.close()
        h.close()
        i.close()
   dist2goal = (nodes_dis[j]*lin_speed)
   angle2goal= (nodes_ang[j]*-.785) + (np.pi/2)

           
   
   






   
   
     





