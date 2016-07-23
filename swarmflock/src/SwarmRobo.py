#!/usr/bin/env python
# Written for indigo

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from swarmflock.msg import BoidMsg, Float32ArrayMsg
from swarmflock.srv import NeighborDiscovery, NeighborDiscoveryResponse, NeighborDiscoveryRequest
from math import radians
from boid import Boid
import numpy as np
import copy, vecutils, math, sys, os, cli
from tf.transformations import euler_from_quaternion
from MonitorAlgo import MonitorAlgo


class SwarmRobo():

  def odom_received(self, msg):
    if(msg.header.seq % 100 == 0):
      self.odom = msg
 

  def msg_received(self, msg):
    # The first boolean expression makes sure we're not including ourselves.
    # The second boolean expression doesn't take a second message from that robot.
    # self.responses will clear when patience expires.

    if(msg.robotName != self.robotName and not
       any(resp.robotName == msg.robotName for resp in self.responses)):
      
      self.responses.append(msg)



  # This is the callback for the patience timer. When this method is called, the robot assumes
  # all communicating members have broadcasted their messages. We now begin to process the information
  # and direct the bot.
  def patience_call(self, event):
    boids = []
    self.neighbors = []

    # Process information as boids
    for resp in self.responses:
      d = np.linalg.norm(self.boid.location - resp.location)

      if d > 0 and d < self.boid.neighR:
        self.neighbors.append(resp)

      nBoid = copy.deepcopy(self.boid)
      nBoid.location = resp.location
      nBoid.velocity = resp.velocity
      boids.append(nBoid)

    if not hasattr(self, 'odom'): 
      rospy.loginfo("Odometry not registered! Returning from patience call.")
      return

    odom = self.odom

    self.boid.location = np.matrix([odom.pose.pose.position.x, odom.pose.pose.position.y])
    oldLocation = self.boid.location
    self.boid.step(boids)


    self.responses = []

    delta = self.boid.velocity
 
    # If there is a current goal, factor it into our navigation.
    if(len(self.goals) > self.currGoal):
      goalDelta = self.goals[self.currGoal] - oldLocation

      if((np.absolute(goalDelta) < self.goalTol).all()):
        rospy.loginfo("Reached goal {0!s}".format(self.goals[self.currGoal]))
        self.currGoal += 1


    # We need to recheck since we might've changed the goal
    if(len(self.goals) > self.currGoal):
      goalDelta = vecutils.limit((self.goals[self.currGoal] - oldLocation) * self.goalWeight, self.maxForce)
      delta  = vecutils.limit(delta + goalDelta, self.maxVelocity)


    dMag = np.linalg.norm(delta)

    # web.engr.oregonstate.edu/~kraftko/code/me456_hw2/lab2_KK.py
    xOr = odom.pose.pose.orientation.x
    yOr = odom.pose.pose.orientation.y
    zOr = odom.pose.pose.orientation.z
    wOr = odom.pose.pose.orientation.w

    (roll, pitch, yaw) = euler_from_quaternion([xOr, yOr, zOr, wOr])
   

    bearingDiff = math.atan2(delta[0, 1], delta[0, 0])
    angle = bearingDiff - yaw

    turn_cmd = Twist()
    nMov_cmd = Twist()

    nMov_cmd.linear.x = dMag
    turn_cmd.angular.z = angle

    self.cmd_vel.publish(turn_cmd)
    rospy.sleep(1)
    self.cmd_vel.publish(nMov_cmd)
    rospy.sleep(1)



  def msgTime_call(self, event):
    msg = BoidMsg()
    msg.robotName = self.robotName
    msg.location = self.boid.location.tolist()[0]
    msg.velocity = self.boid.velocity.tolist()[0]
    self.boid_pub.publish(msg) 


  def goal_received(self, msg):
    self.goals.append(msg.array)



  def discover(self, event):
    members = [x for x in cli.execute_shell('rostopic list | grep swarmflock/boids').split('\n') if x.find(self.robotName) == -1]

    for x in [x for x in members if x not in self.members and x != '']:
      rospy.loginfo("Discovered member %s" % x)
      self.members.append(x)
      self.boid_subs.append(rospy.Subscriber(x, BoidMsg, self.msg_received))



  def __init__(self):
    hostname = os.getenv('HOSTNAME')

    self.robotName = hostname
    self.responses = []
    self.goals = []
    self.currGoal = 0
    self.members = []
    self.boid_subs = []

    # Initiliaze
    rospy.init_node('swarmmember_' + self.robotName, anonymous=False)


    # What to do on CTRL + C    
    rospy.on_shutdown(self.shutdown)


    self.neighborDiscovery = rospy.Service('/' + self.robotName + '/neighbor_discovery', NeighborDiscovery, self.handle_nd)

    
    # Setup communication channels
    self.cmd_vel  = rospy.Publisher('/' + self.robotName + '/cmd_vel_mux/input/navi', Twist, queue_size=10)
    self.odom_sub = rospy.Subscriber('/' + self.robotName + '/odom', Odometry, self.odom_received)
    self.boid_pub = rospy.Publisher('/' + self.robotName + '/swarmflock/boids', BoidMsg, queue_size=10)        
    #self.boid_sub = rospy.Subscriber('/swarmflock/boids', BoidMsg, self.msg_received)
    self.goal_sub = rospy.Subscriber('/swarmflock/goals', Float32ArrayMsg, self.goal_received)


    # Grab global parameters
    self.maxVelocity = float(rospy.get_param("/swarmflock/params/maxVelocity"))
    self.maxForce    = float(rospy.get_param("/swarmflock/params/maxForce"))
    self.desiredSep  = float(rospy.get_param("/swarmflock/params/desiredSep"))
    self.neighR      = float(rospy.get_param("/swarmflock/params/neighborRadius"))
    self.sepWeight   = float(rospy.get_param("/swarmflock/params/sepWeight"))
    self.alignWeight = float(rospy.get_param("/swarmflock/params/alignWeight"))
    self.cohWeight   = float(rospy.get_param("/swarmflock/params/cohWeight"))
    self.goalWeight  = float(rospy.get_param("/swarmflock/params/goalWeight"))
    self.goalTol     = float(rospy.get_param("/swarmflock/params/goalTolerance"))


    # Grab current location from odometry
    rospy.sleep(4)

    if(hasattr(self, 'odom')):
      location = np.matrix([self.odom.pose.pose.position.x, self.odom.pose.pose.position.y])
    else:
      rospy.loginfo('No response from odometry; randomizing Boid position')
      location = np.random.uniform(-250, 250, size=(1,2))


    # Create Boid representation
    self.boid = Boid(location, self.maxVelocity, self.maxForce, self.desiredSep, self.neighR, self.sepWeight, self.alignWeight, self.cohWeight)

    self.discoverTimer = rospy.Timer(rospy.Duration(1), self.discover)
    self.patience = rospy.Timer(rospy.Duration(2), self.patience_call)
    self.msgTimer = rospy.Timer(rospy.Duration(0.4), self.msgTime_call)

    self.monitor = MonitorAlgo(self.robotName, self.boid)

    rospy.loginfo("Now spinning " + self.robotName)
    rospy.spin()




  def shutdown(self):
    # Stop turtlebot
    rospy.loginfo("Stopping member " + self.robotName)
    self.cmd_vel.publish(Twist())
    rospy.sleep(1)


  def handle_nd(self, req):
    resp = NeighborDiscoveryResponse()
    resp.inNeighborhood = req.robotName in [neighbor.robotName for neighbor in self.neighbors]

    msg = BoidMsg()
    msg.robotName = self.robotName
    msg.location = self.boid.location.tolist()[0]
    msg.velocity = self.boid.velocity.tolist()[0]

    resp.boid = msg

    return resp
 


if __name__ == '__main__':
  try:
    SwarmRobo()
  except rospy.exceptions.ROSInterruptException as ex:
    rospy.loginfo("Node terminated.")


