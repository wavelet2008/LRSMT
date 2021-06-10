#!/bin/bash
# ***********************************************
# Author: Shipeng Liu
# Date: 2021-06-09
# Email: shipengl@usc.edu

docker run -it -v ~/Robot-Simulator-In-Granular-Area/roboterrain:/home/roboterrain roboterrain:$HOSTNAME -w /home/roboterrain /bin/bash
