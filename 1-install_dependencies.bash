#!/bin/bash

sudo apt-get install -y git python3 awscli vim repo curl

sudo dpkg -i ./dependencies/git-lfs_2.12.1_amd64.deb
sudo apt-get install -f
git lfs install
git lfs pull
