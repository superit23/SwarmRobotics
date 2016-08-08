SwarmFlock for ROS

# Overview

## Execution Environment

ROS Distribution:	Indigo

Robot model:		Turtlebot 2

Linux Distribution:	Ubuntu 14.04

## Implementation

SwarmFlock is a Python implementation of the boid-flocking algorithm on the Robot Operating System (ROS) with anomaly detection. SwarmFlock currently uses ROS topics over a multi-master system to communicate between the robots. The robots will still only compute "boids" in their neighborhood.

More information on the flocking algorithm can be read [here](http://harry.me/blog/2011/02/17/neat-algorithms-flocking/).

## Important Scripts

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

#### /HOSTNAME/cmd_vel_mux/input/navi

Communication Type:	Publisher

Message Type:	geometry_msgs/Twist

Used to input movement commands.

#### /HOSTNAME/odom

Communication Type:	Subscriber

Message Type:	nav_msgs/Odometry

Used to read positional data.

#### /HOSTNAME/swarmflock/boids

Communication Type:	Both

Message Type:	swarmflock/BoidMsg

Used to input and read the flocking information from the other robots’ Boid instance.

#### /swarmflock/goals

Communication Type:	Subscriber

Message Type:	swarmflock/Float32ArrayMsg

Used to read navigation goals for the swarm.

# Installation

1. Install prerequisites for ROS and Catkin.

2. Create a [workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)

3. Download SwarmFlock and copy the 'swarmflock' folder into your Catkin workspace.


```sh
git clone https://github.com/superit23/SwarmRobotics.git
cp -R SwarmRobotics/swarmflock ~/catkin_ws/src
```


4. Run the setup script.

```sh
sh ~/catkin_ws/src/swarmflock/src/setup.sh
```


5. While in the Catkin workspace, build and install SwarmFlock

```sh
catkin_make install
```


6. Set up environment variables

```sh
source ~/catkin_ws/install/setup.bash
```


# Running SwarmFlock

1. Edit the parameter values in boidStart.py and set environment variables. If you're using the dependencies.launch file, you need to set ROS_IP, ROS_MASTER_URI, and HOSTNAME. Note: echoing $HOSTNAME will echo your hostname *even if not set*. Explicitly set this.

2. Source ~/catkin_ws/*devel*/setup.bash, and run the Turtlebot:

```sh
roslaunch swarmflock dependencies.launch
```

3. Run the following command on each Turtlebot (or in different terminals):


```sh
rosrun swarmflock boidStart.py
```


4. Send goals to the swarm by publishing a float array to */swarmflock/goals*. Example:

```sh
rostopic pub /swarmflock/goals swarmflock/Float32ArrayMsg '[x, y]'
```


5. The robots will attempt to use Wi-Fi Trilateration servers and flock. Use CTRL + C when you want to stop



# Running a Wi-Fi Trilateration Server

1. Same as 'Running SwarmFlock's 1 and 2.

2. Run the WiFiTrilatSrv.py script with these params:

```sh
rosrun swarmflock WiFiTrilatSrv.py MON_INT CON_INT FREQ SSID PASS IP NETMASK
```

where
MON_INT is the interface you're monitoring on (as in Monitor mode for Wi-Fi),
CON_INT is the interface you'll use to connect to the network,
FREQ is the frequency you'll be using,
SSID is the SSID of the AP/network you'll be connecting to,
PASS is the password for that network,
IP is the IP address you'll use, and
NETMASK is the netmask to complement the IP address.

We need to connect to the network with our Wi-Fi, so we can send out pings as heartbeats. This will allow the other servers to capture the packets in Monitor mode and determine our position. Remember, you need an interface to connect to with ROS as well. A typical setup may have the Wi-Fi Trilateration servers running ROS networking over wired interface and monitoring/sending heartbeats over Wi-Fi.
