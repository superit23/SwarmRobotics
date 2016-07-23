#!/usr/bin/python

import rospy
import numpy as np
import math
import time, copy
from swarmflock.msg import BoidMsg
from WiFiTrilatClient import WiFiTrilatClient
from swarmflock.srv import NeighborDiscovery, NeighborDiscoveryResponse
from boid import Boid

class DetectionAlgo:

  def __init__(self, suspect, baseBoid):
    self.posThreshold = np.array([1,1])
    self.timeThreshold = 2
    self.lastCheckIn = time.time()
    self.lastMsg = Twist()
    self.boid = copy.deepcopy(baseBoid)

    self.boidSub = rospy.Subscriber('/' + self.suspect + '/swarmflock/boids', BoidMsg, self.handle_msg)
    self.client = WiFiTrilatClient()

    # Suspect variables
    self.suspectMAC = self.client.hostToIP(self.client.IPtoMAC(self.suspect))
    self.suspect = suspect
    self.suspectVelocity = np.array([0,0])
    self.suspectPos = np.array([0,0])
    self.suspicious = False

    self.runTimer = rospy.Timer(rospy.Duration(1), self.run)



  def handle_msg(self, msg):
    self.lastCheckIn = time.time()
    self.lastMsg = msg


  def run(self, event):
    broadcastedPos = np.array(self.lastMsg.location)
    suspectPos = np.array(self.client.trilaterate(self.suspectMAC, self.client.discover()))

    self.suspectVel = suspectPos - self.suspectPos
    self.suspectPos = suspectPos

    shouldBePos = calcShouldBePos(getNeighbors())

    suspicious = np.abs(suspectPos - broadcastedPos) > self.posThreshold
    suspicious = np.abs(suspectPos - shouldBePos) > self.posThreshold or suspicious
    suspicious = math.abs(time.time() - self.lastCheckIn) > self.timeThreshold or suspicious
    self.suspicious = suspicious or self.suspicious



  def getNeighbors(self):
    servers = [x for x in cli.execute_shell('rostopic list | grep neighbor_discovery').split('\n')]
    services = [rospy.ServiceProxy(x, NeighborDiscovery) for x in servers]

    if len(services > 1):
      responses = [service(self.suspect) for service in services]
    else:
      responses = services(self.suspect)

    return responses




  def calcShouldBePos(self, responses):
    self.boid.location = suspectPos
    self.boid.velocity = self.suspectVelocity

    boids = []

    for resp in responses:
      nBoid = copy.deepcopy(self.boid)
      nBoid.location = resp.location
      nBoid.velocity = resp.velocity
      boids.append(nBoid)

    self.boid.step(boids)

    return self.boid.location
