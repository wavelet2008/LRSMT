#!/bin/bash
# ***********************************************
# Author: Shipeng Liu
# Date: 2021-06-09
# Email: shipengl@usc.edu
# ***********************************************
sudo apt-get update
sudo apt-get install -y \
     apt-transport-https \
     ca-certificates \
     curl \
     gnupg-agent \
     software-properties-common
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -
sudo add-apt-repository \
     "deb [arch=amd64] https://download.docker.com/linux/ubuntu \
     $(lsb_release -cs) \
     stable"
sudo apt-get update
sudo apt-get install docker-ce docker-ce-cli containerd.io
sudo service docker start
sudo docker run hello-world
sudo usermod -aG docker shipengl
sudo service docker restart


