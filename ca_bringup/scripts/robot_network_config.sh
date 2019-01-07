#!/bin/bash

# usage: source robot_network_config.sh

ROBOT_IP=$(ip a s|sed -ne '/127.0.0.1/!{s/^[ \t]*inet[ \t]*\([0-9.]\+\)\/.*$/\1/p}')

echo "Configuring ROS Networking on Master [IP: $ROBOT_IP]"

# Master only cares about host IP
export ROS_MASTER_URI=http://$ROBOT_IP:11311
export ROS_IP=$ROBOT_IP
export ROS_HOSTNAME=$ROBOT_IP

echo "ROS Master Networking configured!"
