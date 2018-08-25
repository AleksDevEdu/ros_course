#!/bin/bash

sudo mount -t vboxsf -o uid=$UID,gid=$(id -g) vm_share ~/host
