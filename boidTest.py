#!/usr/bin/python
import boid
import numpy as np
from matplotlib import pyplot as plt
from matplotlib import animation

fig = plt.figure()
ax = plt.axes(xlim=(-500, 500), ylim=(-500, 500))
pnts, = ax.plot([], [], marker='o', ms=10, linestyle='None')
pnts.set_data([], [])


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


def animate(i):

  boidsMat = np.zeros((0,2))

  for boid in boids:
    boid.step(boids)
    boidsMat = np.concatenate((boidsMat, boid.location))

  pnts.set_data(boidsMat[:, 0], boidsMat[:, 1])
  pnts.set_markersize(10)
  return pnts,
    

anim = animation.FuncAnimation(fig, animate, frames=600, interval=20)
anim.save('boids_test.mp4', fps=30)
