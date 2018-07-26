#!/bin/bash

IP_ADDR=$(ifconfig enp3s0 | grep 'inet addr:' | cut -d: -f2 | awk '{ print $1}')

echo "Set ROS_IP variable to $IP_ADDR"
export ROS_IP=$IP_ADDR
