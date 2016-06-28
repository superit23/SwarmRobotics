#!/usr/bin/python
import boid
import numpy as np

print "Hello"
maxSpeed = 4
maxForce = 0.5
desiredSep = 100
neighborRadius = 150
sepWeight = 10
alignWeight = 0.1
cohWeight = 0.2

boids = []

for i in range(40):
  location = np.random.uniform(-250, 250, size=(1,2))
  boids.append(boid.Boid(location, maxSpeed, maxForce, desiredSep, neighborRadius, sepWeight, alignWeight, cohWeight))

for i in range(100):
  for boid in boids:
    boid.step(boids)

for boid in boids:
  print boid.location
