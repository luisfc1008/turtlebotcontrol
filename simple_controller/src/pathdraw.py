#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

path = Path()

def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass

class NavPath(object):
    def __init__(self):
        self._path = []

    def callback(self, msg):
        rospy.loginfo(msg)
        if SOME_CONDITION:
            self._path.append(msg.foo.bar)
            
def main():
    # ...setup stuff...
    nav_path = odom_cb()
    rospy.Subscriber('/odom', Odometry, odom_cb)
    rospy.spin()

def odom_cb(data):
    global path
    path.header = data.header
    pose = PoseStamped()
    pose.header = data.header
    pose.pose = data.pose.pose
    path.poses.append(pose)
    path_pub.publish(path)
    
rospy.init_node('path_node')

def show_text_in_rviz(marker_publisher, text):
    marker = Marker(
                type=Marker.LINE_STRIP,
                id=0,
                lifetime=rospy.Duration(1.5),
                pose=Pose(Point(0.5, 0.5, 1.45), Quaternion(0, 0, 0, 1)),
                scale=Vector3(0.6, 0.6, 0.6),
                header=Header(frame_id='odom'),
                color=ColorRGBA(0.0, 1.0, 0.0, 0.8),
                text=text)
    marker_publisher.publish(marker)
    
odom_sub = rospy.Subscriber('/odom', Odometry, odom_cb)
marker_publisher = rospy.Publisher('/visualization_marker', Marker, queue_size=5)

if __name__ == '__main__':
    main()
    


