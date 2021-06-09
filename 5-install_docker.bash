#!/bin/bash

declare user_name=`whoami`

sudo apt-get remove docker docker-engine docker.io containerd runc
sudo apt-get install apt-transport-https \
     ca-certificates \
     curl \
     gnupg2 \
     software-properties-common

sudo dpkg -i ./dependencies/docker-ce_5%3a19.03.13~3-0~ubuntu-xenial_amd64.deb
sudo apt-get install -f

sudo addgroup docker
sudo usermod -aG docker ${user_name}

if grep -q docker /etc/group
    then
        echo "group docker exists"
    else
        newgrp - docker
    fi

sudo systemctl unmask docker.service
sudo systemctl unmask docker.socket
sudo systemctl start docker.service
sudo service docker restart

sudo docker run hello-world

docker volume ls -q -f driver=nvidia-docker | \
       xargs -r -I{} -n1 docker ps -q -a -f volume={} | \
       xargs -r docker rm -f
sudo apt-get purge -y nvidia-docker2

sudo dpkg -i ./dependencies/libnvidia-container1_1.3.0-1_amd64.deb
sudo dpkg -i ./dependencies/libnvidia-container-tools_1.3.0-1_amd64.deb
sudo dpkg -i ./dependencies/nvidia-container-runtime_3.4.0-1_amd64.deb
sudo dpkg -i ./dependencies/nvidia-container-toolkit_1.3.0-1_amd64.deb
sudo dpkg -i ./dependencies/nvidia-docker2_2.5.0-1_all.deb
sudo apt-get install -f

sudo service docker restart

docker_run_test="docker run -it --rm --privileged --runtime=nvidia nvidia/cuda:9.0-base nvidia-smi"
echo "$docker_run_test"
bash -cl $docker_run_test
echo "=== start a fake login bash, in order to make docker group activate, pls reboot ==="
bash -cl bash
