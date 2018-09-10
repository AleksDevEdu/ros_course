#!/bin/bash

mkdir -p ~/host
sudo mount -t vboxsf -o uid=$UID,gid=$(id -g) vm_share ~/host
