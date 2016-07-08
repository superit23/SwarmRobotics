#!/usr/bin/python
import boid
import numpy as np
import os
import rospy
import sys
from nRobo import SwarmRobo


maxSpeed = 0.65
maxForce = 0.24
desiredSep = 3
neighborRadius = 10
sepWeight = 0.5
alignWeight = 0.6
cohWeight = 0.8

rospy.set_param("/boids/maxSpeed", str(maxSpeed))
rospy.set_param("/boids/maxForce", str(maxForce))
rospy.set_param("/boids/desiredSep",  str(desiredSep))
rospy.set_param("/boids/neighborRadius", str(neighborRadius))
rospy.set_param("/boids/sepWeight", str(sepWeight))
rospy.set_param("/boids/alignWeight", str(alignWeight))
rospy.set_param("/boids/cohWeight", str(cohWeight))

procs = []

for i in range(1, len(sys.argv)):
  #os.system("./nRobo.py " + str(sys.argv[i]) + " " + str(location))
  print "Initializing " + sys.argv[i]
  SwarmRobo(sys.argv[i], [(20, 20)])
  #p = Process(target=SwarmRobo, args=(sys.argv[i],))
  #p.start()
  #procs.append(p)

#print "Now spinning..."
#try:
#  while True:
#    time.sleep(1)

#except KeyboardInterrupt:
#  pass

#for p in procs:
#  print "Terminating process..."
#  p.terminate() 
