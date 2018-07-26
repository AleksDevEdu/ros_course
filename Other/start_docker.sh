#!/bin/bash

docker run --rm --name 'ros_container' --net=host -ti -v $(pwd):/home/developer kail4ek/ros_container
