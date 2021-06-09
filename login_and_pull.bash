#!/bin/bash

declare cur_dir=`dirname $(readlink -ef $0)`

sudo cp /etc/docker/daemon.json /etc/docker/daemon.json.bak
sudo cp ${cur_dir}/daemon.json /etc/docker/

declare hosts=`cat /etc/hosts | grep "harbor.senseauto.com"`
if [[ -z ${hosts} ]];
then
    echo "10.53.7.11     harbor.senseauto.com" | sudo tee -a /etc/hosts > /dev/null
fi

sudo systemctl daemon-reload
sudo systemctl restart docker

docker login -u developer -p Goodsense@2019 harbor.senseauto.com

bash ${cur_dir}/pull_docker_image.bash
