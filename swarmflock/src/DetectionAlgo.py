#!/usr/bin/python

import rospy
import numpy as np
import math
import time
from swarmflock.msg import BoidMsg
from WiFiTrilatClient import WiFiTrilatClient

class DetectionAlgo:

  def __init__(self, suspect):
    self.posThreshold = np.array([1,1])
    self.timeThreshold = 2
    self.lastCheckIn = time.time()
    self.lastMsg = Twist()
    self.suspect = suspect

    self.boidSub = rospy.Subscriber('/' + self.suspect + '/swarmflock/boids', BoidMsg, self.handle_msg)
    self.client = WiFiTrilatClient()
    self.suspectMAC = self.client.hostToIP(self.client.IPtoMAC(self.suspect))


  def handle_msg(self, msg):
    self.lastCheckIn = time.time()
    self.lastMsg = msg


  def run(self, event):
    broadcastedPos = np.array(self.lastMsg.location)
    suspectPos = np.array(self.client.trilaterate(self.suspectMAC, self.client.discover()))
    shouldBePos = # Neighbor discovery algorithm

    suspicious = np.abs(suspectPos - broadcastedPos) > self.posThreshold
    suspicious = np.abs(suspectPos - shouldBePos) > self.posThreshold or suspicious
    suspicious = math.abs(time.time() - self.lastCheckIn) > self.timeThreshold or suspicious
    return suspicious
