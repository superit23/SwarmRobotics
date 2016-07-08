#!/usr/bin/python
import boid
import numpy as np
import os
import rospy
import sys
from SwarmRobo import SwarmRobo


maxVelocity = 0.65
maxForce = 0.24
desiredSep = 3
neighborRadius = 10
sepWeight = 0.5
alignWeight = 0.6
cohWeight = 0.8
goalWeight = 1.5
goalTolerance = 1


rospy.set_param("/swarmflock/params/maxVelocity", str(maxVelocity))
rospy.set_param("/swarmflock/params/maxForce", str(maxForce))
rospy.set_param("/swarmflock/params/desiredSep",  str(desiredSep))
rospy.set_param("/swarmflock/params/neighborRadius", str(neighborRadius))
rospy.set_param("/swarmflock/params/sepWeight", str(sepWeight))
rospy.set_param("/swarmflock/params/alignWeight", str(alignWeight))
rospy.set_param("/swarmflock/params/cohWeight", str(cohWeight))
rospy.set_param("/swarmflock/params/goalWeight", str(goalWeight))
rospy.set_param("/swarmflock/params/goalTolerance", str(goalTolerance))

procs = []

for i in range(1, len(sys.argv)):
  #os.system("./nRobo.py " + str(sys.argv[i]) + " " + str(location))
  print "Initializing " + sys.argv[i]
  SwarmRobo(sys.argv[i])
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
