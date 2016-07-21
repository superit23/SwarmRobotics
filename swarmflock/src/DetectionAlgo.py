#!/usr/bin/python

import rospy
import numpy as np
import math
import time
from geometry_msgs.msg import Twist
from WiFiTrilatClient import WiFiTrilatCient

class DetectionAlgo:

  def __init__(self, suspect):
    self.posThreshold = np.array([1,1])
    self.timeThreshold = 2
    self.lastCheckIn = time.time()
    self.lastMsg = Twist()
    self.suspect = suspect

    self.odomSub = rospy.Subscriber('/' + self.suspect + '/odom', Twist, self.handle_msg)



  def handle_msg(self, msg):
    self.lastCheckIn = time.time()
    self.lastMsg = msg


  def run(self, event):
    pos = self.lastMsg.position.position
    broadcastedPos = np.array([pos.x, pos.y])


    suspicious = np.abs(suspectPos - broadcastedPos) > self.posThreshold
    suspicious = np.abs(suspectPos - shouldBePos) > self.posThreshold or suspicious
    suspicious = math.abs(time.time() - self.lastCheckIn) > self.timeThreshold or suspicious
    return suspicious
