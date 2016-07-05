#!/usr/bin/python
import boid
import numpy as np
import os
import rospy
import sys
from nRobo import SwarmRobo

maxSpeed = 4
maxForce = 0.5
desiredSep = 100
neighborRadius = 150
sepWeight = 10
alignWeight = 0.1
cohWeight = 0.2

rospy.set_param("/boids/maxSpeed", str(maxSpeed))
rospy.set_param("/boids/maxForce", str(maxForce))
rospy.set_param("/boids/desiredSep",  str(desiredSep))
rospy.set_param("/boids/neighborRadius", str(neighborRadius))
rospy.set_param("/boids/sepWeight", str(sepWeight))
rospy.set_param("/boids/alignWeight", str(alignWeight))
rospy.set_param("/boids/cohWeight", str(cohWeight))

for i in range(1, len(sys.argv)):
  #os.system("./nRobo.py " + str(sys.argv[i]) + " " + str(location))
  SwarmRobo(sys.argv[i])

#boids = []

#for i in range(40):
#  location = np.random.uniform(-250, 250, size=(1,2))
#  boids.append(boid.Boid(location, maxSpeed, maxForce, desiredSep, neighborRadius, sepWeight, alignWeight, cohWeight))

#for i in range(100):
#  for boid in boids:
#    boid.step(boids)

#for boid in boids:
#  print boid.location
