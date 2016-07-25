#!/usr/bin/python

import rospy, cli
import numpy as np
from DetectionAlgo import DetectionAlgo
from swarmflock.msg import ClaimMsg
from collections import Counter

class MonitorAlgo:

  def __init__(self, robotName, baseBoid, suspect=""):
    self.robotName = robotName

    self.claimPub = rospy.Publisher('/swarmflock/claims', ClaimMsg, queue_size=10)
    self.claimSub = rospy.Subscriber('/swarmflock/claims', ClaimMsg, self.handle_claim)
    self.claims = []
    self.myClaim = ""

    self.timer = rospy.Timer(rospy.Duration(15), self.reset_suspect)

    self.manualSuspect = suspect



  def discover(self):
    members = [x[1:x.find('/swarmflock')] for x in cli.execute_shell('rostopic list | grep swarmflock/boids').split('\n')]
    return members



  def selectSuspect(self):
    members = self.discover()

    # notClaimed is a list of all members that have not been claimed and is not the current robot.
    notClaimed = sorted([member for member in members if (member not in [claim.suspect for claim in self.claims]
                 and member != self.robotName)])

    # There are n members, and each member distinctly claims one member. Therefore, if there
    # are not any members left to monitor, there is an anomaly.

    if len(notClaimed) > 0:
      if self.myClaim == "":
        self.myClaim = self.robotName

      # This takes the next robot relative to the last suspect or, when it's the first time, itself.
      index = (notClaimed.index(self.myClaim) + 1) % len(notClaimed)


      self.myClaim = notClaimed[index]
      claim = ClaimMsg()
      claim.claimer = self.robotName
      claim.suspect = notClaimed[index]

      self.claimPub.publish(claim)
    else:
      suspicious = [key for key, value in Counter([claim.claimer for claim in self.claims]).most_common() if value > 1]

      for anomaly in suspicious:
        rospy.loginfo("%s IS ANOMALOUS: This unit has claimed more than one suspect!")



  def reset_suspect(self, event):

    # If we haven't asked for a suspect, go through selection process.
    # If we have asked for a suspect, but we've already watched them, stop monitoring them.
    # This is to enable a one-time, third-party confirmation of an anomalous robot.
    # Otherwise, this must be the first round of monitoring for a third-party confirmation.
    if self.manualSuspect == "":
      self.selectSuspect()
    elif self.manualSuspect == self.myClaim:
      self.timer.shutdown()
      self.dAlgo = None
      return
    else:
      self.myClaim = self.manualSuspect

    self.dAlgo = DetectionAlgo(self.myClaim, baseBoid)





  def handle_claim(self, claim):
    #if(claim.suspect == self.suspect and claim.claimer != self.robotName):
    #  self.claimPub()
    self.claims.append(claim)



  def handle_suspicion(self, boidMsg):
    self.confirmation = MonitorAlgo(self.robotName, self.boid, boidMsg.robotName)
