#!/usr/bin/python

import rospy
import numpy as np
import math
import time, copy, cli
from swarmflock.msg import BoidMsg, SuspicionMsg
from WiFiTrilatClient import WiFiTrilatClient
from swarmflock.srv import NeighborDiscovery, NeighborDiscoveryResponse
from boid import Boid


class DetectionAlgo:

  def __init__(self, robotName, isConfirmation, suspect, baseBoid):
    self.posThreshold = np.array([1,1])
    self.timeThreshold = 2
    self.lastCheckIn = time.time()
    self.lastMsg = None
    self.boid = copy.deepcopy(baseBoid)

    self.robotName = robotName
    self.suspect = suspect
    self.isConfirm = isConfirmation

    self.suspicionPub = rospy.Publisher('/swarmflock/suspicion', SuspicionMsg, queue_size=10)
    self.boidSub = rospy.Subscriber('/' + self.suspect + '/swarmflock/boids', BoidMsg, self.handle_msg)
    self.client = WiFiTrilatClient()

    # Suspect variables
    self.suspectMAC = self.client.hostToIP(self.client.IPtoMAC(self.suspect))
    self.suspectVel = np.array([0,0])
    self.suspectPos = np.array([0,0])
    self.suspicious = False


    self.runTimer = rospy.Timer(rospy.Duration(1), self.run)



  def handle_msg(self, msg):
    self.lastCheckIn = time.time()
    self.lastMsg = msg
 

  def run(self, event):
    broadcastedPos = np.array(self.lastMsg.location)
    suspectPos = np.array(self.client.trilaterate(self.suspectMAC, self.client.discover(), time.time()))

    self.suspectVel = suspectPos - self.suspectPos
    self.suspectPos = suspectPos

    shouldBePos = self.calcShouldBePos(self.getNeighbors())

    liedAboutPos = np.abs(suspectPos - broadcastedPos) > self.posThreshold
    wrongPos = np.abs(suspectPos - shouldBePos) > self.posThreshold
    stoppedTalking = math.fabs(time.time() - self.lastCheckIn) > self.timeThreshold

    allReasons = ""

    if liedAboutPos:
      allReasons += "Lied about its position!\n"

    if wrongPos:
      allReasons += "Is in wrong position!\n"

    if stoppedTalking:
      allReasons += "Has stopped talking!\n"



    self.suspicious = liedAboutPos or wrongPos or stoppedTalking or self.suspicious

    if self.suspicious:
      rospy.logwarn("%s IS ANOMALOUS: %s" % (self.suspect, allReasons))


    #self.suspicious = stoppedTalking or self.suspicious

    if self.suspicious:
      suspMsg = SuspicionMsg()
      suspMsg.robotName = self.robotName
      suspMsg.reason = allReasons
      suspMsg.isConfirmation = self.isConfirm
      suspMsg.boid = self.lastMsg


      self.suspicionPub.publish(suspMsg)



  def getNeighbors(self):
    servers = [x for x in cli.execute_shell('rosservice list | grep neighbor_discovery').split('\n') if x != '']
    services = [rospy.ServiceProxy(x, NeighborDiscovery) for x in servers]


    #if len(services) > 1:
    responses = [service(self.suspect) for service in services]
    #else:
    #  responses = services(self.suspect)

    return responses




  def calcShouldBePos(self, responses):
    self.boid.location = self.suspectPos
    self.boid.velocity = self.suspectVel

    boids = []

    for resp in responses:
      nBoid = copy.deepcopy(self.boid)
      nBoid.location = resp.boid.location
      nBoid.velocity = resp.boid.velocity
      boids.append(nBoid)

    self.boid.step(boids)

    return self.boid.location
