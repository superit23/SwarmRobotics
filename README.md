SwarmFlock for ROS

# Overview

## Execution Environment

ROS Distribution:	Indigo

Robot model:		Turtlebot 2

Linux Distribution:	Ubuntu 14.04

## Implementation

SwarmFlock is a Python implementation of the boid-flocking algorithm on the Robot Operating System (ROS). SwarmFlock currently uses ROS topics to communicate between the robots, and, therefore, is more of a simulation of swarm intelligence due to centralized communication. The robots will still only compute "boids" in their neighborhood.

More information on the flocking algorithm can be read [here](http://harry.me/blog/2011/02/17/neat-algorithms-flocking/).

## Scripts

### boid.py

Contains the Boid class and is where the flocking algorithm is run.

### SwarmRobo.py

Contains the robot’s actual implementation.

### boidStart.py

Convenience script to create a swarm member. Takes the member’s name as an argument.

## ROS integration

### Parameters

#### /swarmflock/params/maxVelocity

Used in: boidStart.py, SwarmRobo.py

This gets passed to the local Boid instance and determines the maximum velocity of that Boid.

#### /swarmflock/params/maxForce

Used in: boidStart.py, SwarmRobo.py

This gets passed to the local Boid instance and determines the maximum force (acceleration) of that Boid.

#### /swarmflock/params/desiredSep

Used in: boidStart.py, SwarmRobo.py

This gets passed to the local Boid instance and determines the desired separation of that Boid in respect to other Boids.

#### /swarmflock/params/neighborRadius

Used in: boidStart.py, SwarmRobo.py

This gets passed to the local Boid instance and determines the radius of the neighborhood of that Boid.

#### /swarmflock/params/sepWeight

Used in: boidStart.py, SwarmRobo.py

This gets passed to the local Boid instance and determines how much weight the separation rule  has with that Boid.

#### /swarmflock/params/alignWeight

Used in: boidStart.py, SwarmRobo.py

This gets passed to the local Boid instance and determines how much weight the alignment rule  has with that Boid.

#### /swarmflock/params/cohWeight

Used in: boidStart.py, SwarmRobo.py

This gets passed to the local Boid instance and determines how much weight the cohesion rule  has with that Boid.

#### /swarmflock/params/goalWeight

Used in: boidStart.py, SwarmRobo.py

This is used in SwarmRobo.py to determine how much weight the "goals" (positions to go to) will have on the robot’s navigation.

#### /swarmflock/params/goalTolerance

Used in: boidStart.py, SwarmRobo.py

This is used in SwarmRobo.py to determine when the robot has reached the goal. For example, if set to 1, the robot will be satisfied if it comes within one meter (in both dimensions) of the goal.

### Topics

#### /ROBOT_NAME/cmd_vel_mux/input/navi

Communication Type:	Publisher

Message Type:	geometry_msgs/Twist

Used to input movement commands.

#### /ROBOT_NAME/odom

Communication Type:	Subscriber

Message Type:	nav_msgs/Odometry

Used to read positional data.

#### /swarmflock/boids

Communication Type:	Both

Message Type:	swarmflock/BoidMsg

Used to input and read the flocking information from the other robots’ Boid instance.

#### /swarmflock/goals

Communication Type:	Subscriber

Message Type:	swarmflock/Float32ArrayMsg

Used to read navigation goals for the swarm.

# Installation

1. Install prerequisites (TODO)

2. Create a [workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)

3. Download SwarmFlock

<table>
  <tr>
    <td>git clone https://github.com/superit23/SwarmRobotics.git</td>
  </tr>
</table>


4. Set up environment variables

<table>
  <tr>
    <td>source ~/catkin_ws/install/setup.bash</td>
  </tr>
</table>


5. While in the Catkin workspace, build and install SwarmFlock

<table>
  <tr>
    <td>catkin_make install</td>
  </tr>
</table>


# Running it

1. Edit the parameter values in boidStart.py.

2. Bring up your Turtlebots. You can use the Gazebo Concert if you want to simulate.

3. Run the following command on each Turtlebot (or in different terminals):

	

<table>
  <tr>
    <td>rosrun swarmflock boidStart.py robotName</td>
  </tr>
</table>


4. Send goals to the swarm by publishing a float array to */swarmflock/goals*. Example:

<table>
  <tr>
    <td>rostopic pub /swarmflock/goals swarmflock/Float32ArrayMsg '[x, y]'</td>
  </tr>
</table>


5. Use CTRL + C when you want to stop

