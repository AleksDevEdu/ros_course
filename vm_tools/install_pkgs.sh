#!/bin/bash

# Manual install:
# - Google chrome: https://www.google.com/intl/ru_ALL/chrome/
# - Roboware: http://www.roboware.me/#/home

# Sublime text 3 preinstall
wget -qO - https://download.sublimetext.com/sublimehq-pub.gpg | sudo apt-key add -
sudo apt-get install apt-transport-https
echo "deb https://download.sublimetext.com/ apt/stable/" | sudo tee /etc/apt/sources.list.d/sublime-text.list

# ROS preinstall
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

# Install everything
sudo apt update
sudo apt install -y git \
					sublime-text \
					ros-melodic-desktop-full python-rosinstall python-rosinstall-generator python-wstool build-essential \

# ROS postinstall
ROS_DEP_SRC_FILE="/etc/ros/rosdep/sources.list.d/20-default.list"
if [ ! -f $ROS_DEP_SRC_FILE ]; then
	sudo rosdep init && rosdep update
else
	echo "rosdep init skipped"
fi

echo 'Set ROS rc variables'
ROS_SETUP='source "/opt/ros/melodic/setup.bash"'
ROS_RC_FILE=$HOME/.bashrc
grep -qF -- "$ROS_SETUP" "$ROS_RC_FILE" || echo $ROS_SETUP >> $ROS_RC_FILE

# Git postinstall
git config --global user.name "ros-vm-user"
git config --global user.email 'ros-vm-user@mail.ru'
