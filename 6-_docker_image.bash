#!/bin/bash

environment_wd=$(pwd)

cd ./repo_pro/senseauto/system/scripts/docker
docker_script_wd=$(pwd)

## After installation of docker, you might need to logout your user account and login again to refresh groups cache.
## If not, you might still need to run 'docker' with command 'sudo'

## Login and pull image: core.harbor.domain/senseauto/developer:latest (Changed 2019-12-26: driveworks-0.6 -> driveworks-2.0)
cd $docker_script_wd
./login_and_pull.bash

## Build your own image
cd $docker_script_wd
./build_owner_docker.sh

cd $environment_wd
