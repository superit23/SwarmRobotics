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
from math import radians
import sys
from boid import Boid
from swarmflock.msg import BoidMsg
import numpy as np

class SwarmRobo():

  def msg_received(self, msg):
    # The first boolean expression makes sure we're not including ourselves.
    # The second boolean expression doesn't take a second message from that robot.
    # self.responses will clear when patience expires.
    # The third boolean expression makes sure we haven't just seen this message.

    if(msg.robotName != self.robotName and not
       any(resp.robotName == msg.robotName for resp in self.responses)):# and not
       #any(msg.header.seq == seq for seq in self.pastSeqs)):
      
      self.responses.append(msg)
      #self.pastSeqs.append(msg.header.seq)
      #print msg
      #print msg.header.seq


  def patience_call(self, event):
    for resp in self.responses:
      print resp

    #self.pastSeqs = self.pastSeqs[len(self.responses) - 1::]
    self.responses = []


  def __init__(self, robotName, location):
    self.robotName = robotName

    # initiliaze
    rospy.init_node('swarmmember_' + self.robotName, anonymous=False)

    # What to do you ctrl + c    
    rospy.on_shutdown(self.shutdown)
      
    self.cmd_vel = rospy.Publisher('/' + self.robotName + '/cmd_vel_mux/input/navi', Twist, queue_size=10)
    self.boid_pub = rospy.Publisher('/boid', BoidMsg, queue_size=10)        
    self.boid_sub = rospy.Subscriber('/boid', BoidMsg, self.msg_received)

    maxSpeed = rospy.get_param("/boids/maxSpeed")
    maxForce = rospy.get_param("/boids/maxForce")
    desiredSep = rospy.get_param("/boids/desiredSep")
    neighR = rospy.get_param("/boids/neighborRadius")
    sepWeight = rospy.get_param("/boids/sepWeight")
    alignWeight = rospy.get_param("/boids/alignWeight")
    cohWeight = rospy.get_param("/boids/cohWeight")

    self.boid = Boid(location, maxSpeed, maxForce, desiredSep, neighR, sepWeight, alignWeight, cohWeight)

    #5 HZ
    r = rospy.Rate(5);

    self.patience = rospy.Timer(rospy.Duration(1), self.patience_call)
    self.responses = []
    self.pastSeqs = []

    # create two different Twist() variables.  One for moving forward.  One for turning 45 degrees.

    # let's go forward at 0.2 m/s
    move_cmd = Twist()
    move_cmd.linear.x = 0.2
    # by default angular.z is 0 so setting this isn't required

    #let's turn at 45 deg/s
    turn_cmd = Twist()
    turn_cmd.linear.x = 0
    turn_cmd.angular.z = radians(45); #45 deg/s in radians/s

    #two keep drawing squares.  Go forward for 2 seconds (10 x 5 HZ) then turn for 2 second
    count = 0
    while not rospy.is_shutdown():
      msg = BoidMsg()
      msg.robotName = self.robotName
      msg.location = self.boid.location.tolist()[0]
      msg.velocity = self.boid.velocity.tolist()[0]
      self.boid_pub.publish(msg)

      r.sleep()



  def shutdown(self):
    # stop turtlebot
    rospy.loginfo("Stopping member " + self.robotName)
    self.cmd_vel.publish(Twist())
    rospy.sleep(1)
 
if __name__ == '__main__':
  try:
    SwarmRobo(sys.argv[1], sys.argv[2])
  except rospy.exceptions.ROSInterruptException as ex:
    rospy.loginfo("node terminated.")


