#!/usr/bin/python
import rospy
import roslib
import tf
from nav_msgs.msg import Odometry

def odomProcess(msg):
  if(msg.header.seq % 200 == 0):
   print msg

if __name__ == "__main__":
  rospy.init_node('odom_reader', anonymous=False)
  rospy.Subscriber('/gamza1/odom', Odometry, odomProcess)
  rospy.spin()

