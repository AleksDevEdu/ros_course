#!/bin/bash

docker run --rm --name 'ros_container' -ti -v $(pwd):/root ros
