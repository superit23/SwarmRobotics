#!/bin/bash

ip=$(hostname -I)

export HOSTNAME=$(hostname)
export ROS_IP=${ip::-1}
export ROS_MASTER_URI=http://$ROS_IP:11311
