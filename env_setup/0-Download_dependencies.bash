#!/bin/bash
# ***********************************************
# Author: Shipeng Liu
# Date: 2021-06-09
# Email: shipengl@usc.edu
# ***********************************************

# install 
sudo apt-get update
sudo apt-get install -y git python3 vim curl

# download dependencies
mkdir dependencies 
wget https://github.com/Ryoma-Liu/Robot-Simulator-In-Granular-Area/releases/download/Dependencies/docker-ce_5.3a19.03.13.3-0.ubuntu-xenial_amd64.deb -P dependencies
wget https://github.com/Ryoma-Liu/Robot-Simulator-In-Granular-Area/releases/download/Dependencies/git-lfs_2.12.1_amd64.deb -o dependencies
wget https://github.com/Ryoma-Liu/Robot-Simulator-In-Granular-Area/releases/download/Dependencies/libcuda1-418_418.56-0ubuntu0.gpu16.04.1_amd64.deb -P dependencies
wget https://github.com/Ryoma-Liu/Robot-Simulator-In-Granular-Area/releases/download/Dependencies/libnvidia-container-tools_1.3.0-1_amd64.deb -P dependencies
wget https://github.com/Ryoma-Liu/Robot-Simulator-In-Granular-Area/releases/download/Dependencies/libnvidia-container1_1.3.0-1_amd64.deb -P dependencies
wget https://github.com/Ryoma-Liu/Robot-Simulator-In-Granular-Area/releases/download/Dependencies/nvidia-418_418.56-0ubuntu0.gpu16.04.1_amd64.deb -P dependencies
wget https://github.com/Ryoma-Liu/Robot-Simulator-In-Granular-Area/releases/download/Dependencies/nvidia-container-runtime_3.4.0-1_amd64.deb -P dependencies
wget https://github.com/Ryoma-Liu/Robot-Simulator-In-Granular-Area/releases/download/Dependencies/nvidia-container-toolkit_1.3.0-1_amd64.deb -P dependencies
wget https://github.com/Ryoma-Liu/Robot-Simulator-In-Granular-Area/releases/download/Dependencies/nvidia-docker2_2.5.0-1_all.deb -P dependencies
wget https://github.com/Ryoma-Liu/Robot-Simulator-In-Granular-Area/releases/download/Dependencies/nvidia-docker2_2.5.0-1_all.deb -P dependencies
wget https://github.com/Ryoma-Liu/Robot-Simulator-In-Granular-Area/releases/download/Dependencies/nvidia-settings_418.56-0ubuntu0.gpu16.04.1_amd64.deb -P dependencies


