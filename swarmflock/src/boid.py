#!/usr/bin/python
import numpy as np

# Derived from harry.me/blog/2011/02/17/neat-algorithms-flocking/
class Boid:
  def __init__(self, location, maxSpeed, maxForce, desiredSep, neighborRadius, sepWeight, alignWeight, cohWeight):
    self.sepW = sepWeight
    self.alignW = alignWeight
    self.cohW = cohWeight

    self.neighR = neighborRadius

    self.maxSpeed = maxSpeed
    self.maxForce = maxForce
    self.desiredSep = desiredSep
    
    self.location = location
    self.velocity = np.random.uniform(-1, 1, size=(1,2))
    return


  # Call to advance frame
  def step(self, neighbors):
    acceleration = self.flock(neighbors)
    #self.velocity = np.minimum(self.velocity + acceleration, self.maxSpeed)
    self.velocity = self.limit(self.velocity + acceleration, self.maxSpeed)
    self.location = self.location + self.velocity
    return


  # Runs entire flocking algorithm; returns weighted sum
  def flock(self, neighbors):
    separation = self.separate(neighbors) * self.sepW
    alignment = self.align(neighbors) * self.alignW
    cohesion = self.cohere(neighbors) * self.cohW
    return separation + alignment + cohesion


  # Returns cohesion component of acceleration
  def cohere(self, neighbors):
    sum = np.zeros((1,2))
    count = 0

    for boid in neighbors:
      # Find Euclidean distance
      d = np.linalg.norm(self.location - boid.location)

      # Only include boids within neighbor radius
      if d > 0 and d < self.neighR:
        sum += boid.location
        count += 1

    if count > 0:
      return self.steerTo(sum / count)
    else:
      return sum


  def steerTo(self, target):
    desired = target - self.location
    
    # Distance from target is magnitude of vector 
    d = np.linalg.norm(desired)

    # Calc steering for d > 0, else return zero vector
    if d > 0:
      desired = desired / np.linalg.norm(desired)

      # Arbitrary damping
      if d < 100.0:
        desired = desired * (self.maxSpeed * (d / 100.0))
      else:
        desired = desired * self.maxSpeed

      steer = desired - self.velocity
      #steer = np.minimum(steer, self.maxForce)
      steer = self.limit(steer, self.maxForce)

    else:
      steer = np.zeros((1, 2))

    return steer


  # Alignment component for acceleration
  def align(self, neighbors):
    mean = np.zeros((1, 2))
    count = 0

    for boid in neighbors:
      d = np.linalg.norm(self.location - boid.location)

      if d > 0 and d < self.neighR:
        mean += boid.velocity
        count += 1

    if count > 0:
      mean = mean / count

    #mean = np.minimum(mean, self.maxForce)
    mean = self.limit(mean, self.maxForce)
    return mean


  # Separation component for acceleration
  def separate(self, neighbors):
    mean = np.zeros((1, 2))
    count = 0

    for boid in neighbors:
      d = np.linalg.norm(self.location - boid.location)

      if d > 0 and d < self.desiredSep:
        mean += (self.location - boid.location) / (d ** 2)
        count += 1

    if count > 0:
      mean = mean / count

    #return np.minimum(mean, self.maxForce)
    return self.limit(mean, self.maxForce)


  def limit(self, vec, max):
    magnitude = np.linalg.norm(vec)

    if magnitude > max:
      return vec / magnitude * max
    else:
      return vec
