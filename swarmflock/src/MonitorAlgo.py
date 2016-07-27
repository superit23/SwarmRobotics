#!/usr/bin/python

import rospy, cli
import numpy as np
from DetectionAlgo import DetectionAlgo
from swarmflock.msg import SuspicionMsg
from collections import Counter

class MonitorAlgo:

  def __init__(self, robotName, baseBoid, suspect=""):
    self.robotName = robotName

    self.suspect = ""
    self.confirmFor = ""

    self.timer = rospy.Timer(rospy.Duration(15), self.reset_suspect)
    self.suspSub = rospy.Subscriber('/swarmflock/suspicion', SuspicionMsg, self.handle_suspicion)
    self.manualSuspect = suspect



  def discover(self):
    members = [x[1:x.find('/swarmflock')] for x in cli.execute_shell('rostopic list | grep swarmflock/boids').split('\n')]
    return members



  def selectSuspect(self):
    members = self.discover()

    # available is a list of all members that is not the current robot.
    available = sorted([member for member in members if member != self.robotName])

    # There are n members, and each member distinctly claims one member. Therefore, if there
    # are not any members left to monitor, there is an anomaly.
    if self.suspect == "":
      self.suspect = self.robotName

    # This takes the next robot relative to the last suspect or, when it's the first time, itself.
    index = (available.index(self.suspect) + 1) % len(available)

    self.suspect = available[index]



  def selectConfirmFor(self):
    members = self.discover()
    available = sorted([member for member in members if member != self.robotName])

    if self.confirmFor == "":
      self.confirmFor = self.robotName

    # This takes the next robot relative to the last suspect or, when it's the first time, itself.
    index = (available.index(self.confirmFor) + 2) % len(available)

    self.confirmFor = available[index]




  def reset_suspect(self, event):

    # If we haven't asked for a suspect, go through selection process.
    # If we have asked for a suspect, but we've already watched them, stop monitoring them.
    # This is to enable a one-time, third-party confirmation of an anomalous robot.
    # Otherwise, this must be the first round of monitoring for a third-party confirmation.
    isConfirm = False

    if self.manualSuspect == "":
      self.selectSuspect()
      self.selectConfirmFor()

    elif self.manualSuspect == self.suspect:
      self.timer.shutdown()
      self.dAlgo = None
      return

    else:
      self.suspect = self.manualSuspect
      isConfirm = True

    self.dAlgo = DetectionAlgo(self.robotName, isConfirm, self.suspect, baseBoid)


  def handle_suspicion(self, suspMsg):
    # If this is the first confirmation or the suspect has not been handled or if the last confirmation on the same is complete,
    # then handle it. This prevents a compromised host constantly reseting the algorithm.
    if suspMsg.robotName == self.confirmFor and not suspMsg.isConfirmation and (not self.confirmation or not (self.confirmation.suspect == suspMsg.boid.robotName and self.confirmation.dAlgo)):
      self.confirmation = MonitorAlgo(self.robotName, self.boid, suspMsg.boid.robotName)

    elif suspMsg.isConfirmation and self.suspect == suspMsg.boid.robotName:
      rospy.logwarn("SUSPICIONS CONFIRMED ON %s" % self.suspect)
