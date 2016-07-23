#!/usr/bin/python

import rospy, cli
import numpy as np
from DetectionAlgo import DetectionAlgo
from swarmflock.msg import ClaimMsg
from collections import Counter

class MonitorAlgo:

  def __init__(self, robotName, baseBoid):
    self.robotName = robotName

    self.claimPub = rospy.Publisher('/swarmflock/claims', ClaimMsg, queue_size=10)
    self.claimSub = rospy.Subscriber('/swarmflock/claims', ClaimMsg, self.handle_claim)
    self.claims = []
    self.myClaim = ""

    selectSuspect()

    self.dAlgo = DetectionAlgo(self.myClaim, baseBoid)



  def discover(self):
    members = [x[1:x.find('/swarmflock')] for x in cli.execute_shell('rostopic list | grep swarmflock/boids').split('\n')]
    return members



  def selectSuspect(self):
    members = discover()
    notClaimed = [member for member in members not in [claim.suspect for claim in self.claims]]

    # There are n members, and each member distinctly claims one member. Therefore, if there
    # are not any members left to monitor, there is an anomaly.
    if len(notClaimed) > 0:
      self.myClaim = notClaimed[0]
      claim = ClaimMsg()
      claim.claimer = self.robotName
      claim.suspect = notClaimed[0]

      self.claimPub.publish(claim)
    else:
      suspicious = [key for key, value in Counter([claim.claimer for claim in self.claims]).most_common() if value > 1]

      for anomaly in suspicious:
        rospy.loginfo("%s IS ANOMALOUS: This unit has claimed more than one suspect!")



  def handle_claim(self, claim):
    #if(claim.suspect == self.suspect and claim.claimer != self.robotName):
    #  self.claimPub()
    self.claims.append(claim)
