#!/bin/sh
# export ROS_MASTER_URI=http://10.10.10.241:11311  # Jackal's hostname
# export ROS_IP=10.10.10.161                       # Your laptop's wireless IP address
export ROS_MASTER_URI=http://192.168.1.101:11311  # Jackal's hostname
export ROS_IP=192.168.1.102                       # Your laptop's wireless IP address
echo "ROS_MASTER_URI=$ROS_MASTER_URI"
echo "ROS_IP=$ROS_IP"

