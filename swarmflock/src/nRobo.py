#!/usr/bin/env python

'''
Copyright (c) 2015, Mark Silliman
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

'''

# An example of TurtleBot 2 drawing a 0.4 meter square.
# Written for indigo

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import radians
import sys
from boid import Boid
from swarmflock.msg import BoidMsg
import numpy as np
import copy

class SwarmRobo():

  def odom_received(self, msg):
    if(msg.header.seq % 100 == 0):
      self.odom = msg
      #print self.odom


  def msg_received(self, msg):
    # The first boolean expression makes sure we're not including ourselves.
    # The second boolean expression doesn't take a second message from that robot.
    # self.responses will clear when patience expires.
    # [DEPRECATED] The third boolean expression makes sure we haven't just seen this message.

    if(msg.robotName != self.robotName and not
       any(resp.robotName == msg.robotName for resp in self.responses)):# and not
       #any(msg.header.seq == seq for seq in self.pastSeqs)):
      
      self.responses.append(msg)
      #self.pastSeqs.append(msg.header.seq)
      #print msg
      #print msg.header.seq

  # This is the callback for the patience timer. When this method is called, the robot assumes
  # all communicating members have broadcasted their messages. We now begin to process the information
  # and direct the bot.
  def patience_call(self, event):
    boids = []

    for resp in self.responses:
      print resp
      nBoid = copy.deepcopy(self.boid)
      nBoid.location = resp.location
      nBoid.velocity = resp.velocity
      boids.append(nBoid)

    self.boid.step(boids)

    #self.pastSeqs = self.pastSeqs[len(self.responses) - 1::]
    self.responses = []



  def __init__(self, robotName):
    self.robotName = robotName

    # Initiliaze
    rospy.init_node('swarmmember_' + self.robotName, anonymous=False)

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

    self.patience = rospy.Timer(rospy.Duration(1), self.patience_call)
    self.responses = []
    #self.pastSeqs = []

    # Let's go forward at 0.2 m/s
    move_cmd = Twist()
    move_cmd.linear.x = 0.2
    # By default angular.z is 0 so setting this isn't required

    # Let's turn at 45 deg/s
    turn_cmd = Twist()
    turn_cmd.linear.x = 0
    turn_cmd.angular.z = radians(45); #45 deg/s in radians/s


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
    SwarmRobo(sys.argv[1], sys.argv[2])
  except rospy.exceptions.ROSInterruptException as ex:
    rospy.loginfo("Node terminated.")


