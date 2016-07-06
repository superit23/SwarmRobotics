#!/usr/bin/env python
# Written for indigo

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
from math import radians
import sys
from boid import Boid
from swarmflock.msg import BoidMsg
import numpy as np
import copy
import vecutils
import tf
from tf.transformations import euler_from_quaternion
import actionlib
from move_base_msgs.msg import *

class SwarmRobo():

  def odom_received(self, msg):
    if(msg.header.seq % 100 == 0):
      self.odom = msg


  def msg_received(self, msg):
    # The first boolean expression makes sure we're not including ourselves.
    # The second boolean expression doesn't take a second message from that robot.
    # self.responses will clear when patience expires.
    # [DEPRECATED] The third boolean expression makes sure we haven't just seen this message.

    if(msg.robotName != self.robotName and not
       any(resp.robotName == msg.robotName for resp in self.responses)):# and not
       #any(msg.header.seq == seq for seq in self.pastSeqs)):
      
      self.responses.append(msg)


  # This is the callback for the patience timer. When this method is called, the robot assumes
  # all communicating members have broadcasted their messages. We now begin to process the information
  # and direct the bot.
  def patience_call(self, event):
    boids = []

    # Process information as boids
    for resp in self.responses:
      print resp
      nBoid = copy.deepcopy(self.boid)
      nBoid.location = resp.location
      nBoid.velocity = resp.velocity
      boids.append(nBoid)

    odom = self.odom

    self.boid.location = np.matrix([odom.pose.pose.position.x, odom.pose.pose.position.y])
    #oldLocation = self.boid.location
    self.boid.step(boids)

    #self.pastSeqs = self.pastSeqs[len(self.responses) - 1::]
    self.responses = []

    #angle = angle_between(location, self.boid.location)
    #delta = oldLocation - self.boid.location
    delta = self.boid.velocity
    #move_cmd = Twist()
    #move_cmd.linear.x = delta[0, 0]
    #move_cmd.linear.y = delta[0, 1]

    # web.engr.oregonstate.edu/~kraftko/code/me456_hw2/lab2_KK.py
    xOr = odom.pose.pose.orientation.x
    yOr = odom.pose.pose.orientation.y
    zOr = odom.pose.pose.orientation.z
    wOr = odom.pose.pose.orientation.w

    (roll, pitch, yaw) = euler_from_quaternion([xOr, yOr, zOr, wOr])
   

    #angle = self.odom.pose.pose.orientation.z - vecutils.angle_between((1, 0), delta.tolist()[0])
    bearingDiff = math.atan2(delta[0, 0], delta[0, 1])
    angle = bearingDiff - yaw
    print angle

    #if(angle > 180):
    #  angle = 360 - angle
    #  clockwise = -1
    #else:
    #  clockwise = 1

    turn_cmd = Twist()
    nMov_cmd = Twist()
    nMov_cmd.linear.x = np.linalg.norm(delta)
    turn_cmd.angular.z = angle

    #self.cmd_vel.publish(move_cmd)
    self.cmd_vel.publish(turn_cmd)
    rospy.sleep(1)
    self.cmd_vel.publish(nMov_cmd)
    rospy.sleep(1)

    #goal = MoveBaseGoal()
    #goal.target_pose.pose.position.x = self.boid.location[0, 0]
    #goal.target_pose.pose.position.y = self.boid.location[0, 1]
    #goal.target_pose.header.frame_id = self.robotName + 'move_it'
    #goal.target_pose.header.stamp = rospy.Time.now()

    #self.sac.wait_for_server()
    #self.sac.send_goal(goal)
    #self.sac.wait_for_result()
    #print self.sac.get_result()



  def __init__(self, robotName):
    self.robotName = robotName
    self.responses = []

    # Initiliaze
    rospy.init_node('swarmmember_' + self.robotName, anonymous=False)
    self.sac = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    # What to do on CTRL + C    
    rospy.on_shutdown(self.shutdown)
    
    # Setup communication channels
    self.cmd_vel  = rospy.Publisher('/' + self.robotName + '/cmd_vel_mux/input/navi', Twist, queue_size=10)
    self.odom_sub = rospy.Subscriber('/' + self.robotName + '/odom', Odometry, self.odom_received)
    self.boid_pub = rospy.Publisher('/boid', BoidMsg, queue_size=10)        
    self.boid_sub = rospy.Subscriber('/boid', BoidMsg, self.msg_received)

    # Grab global parameters
    self.maxSpeed    = float(rospy.get_param("/boids/maxSpeed"))
    self.maxForce    = float(rospy.get_param("/boids/maxForce"))
    self.desiredSep  = float(rospy.get_param("/boids/desiredSep"))
    self.neighR      = float(rospy.get_param("/boids/neighborRadius"))
    self.sepWeight   = float(rospy.get_param("/boids/sepWeight"))
    self.alignWeight = float(rospy.get_param("/boids/alignWeight"))
    self.cohWeight   = float(rospy.get_param("/boids/cohWeight"))


    # Grab current location from odometry
    rospy.sleep(1)

    if(hasattr(self, 'odom')):
      location = np.matrix([self.odom.pose.pose.position.x, self.odom.pose.pose.position.y])
    else:
      rospy.loginfo('No response from odometry; randomizing Boid position')
      location = np.random.uniform(-250, 250, size=(1,2))

    # Create Boid representation
    self.boid = Boid(location, self.maxSpeed, self.maxForce, self.desiredSep, self.neighR, self.sepWeight, self.alignWeight, self.cohWeight)

    # 5 Hz
    r = rospy.Rate(5);

    self.patience = rospy.Timer(rospy.Duration(2), self.patience_call)

    while not rospy.is_shutdown():
      msg = BoidMsg()
      msg.robotName = self.robotName
      msg.location = self.boid.location.tolist()[0]
      msg.velocity = self.boid.velocity.tolist()[0]
      self.boid_pub.publish(msg)

      r.sleep()



  def shutdown(self):
    # Stop turtlebot
    rospy.loginfo("Stopping member " + self.robotName)
    self.cmd_vel.publish(Twist())
    rospy.sleep(1)
 


if __name__ == '__main__':
  try:
    SwarmRobo(sys.argv[1])
  except rospy.exceptions.ROSInterruptException as ex:
    rospy.loginfo("Node terminated.")


