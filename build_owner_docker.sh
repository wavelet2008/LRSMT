#!/bin/bash
# Copyright (C) 2019 by SenseTime Group Limited. All rights reserved.
# This script will build a new image from base image specified by args '--source-image'.
# In the container created by new image, you will have same permssion with local host user.
# And will copy some config files, such as: ~/.gitconfig, ~/.ssh, into new image.

set -e

SCRIPT_VERSION="1.0.0"
SCRIPT_ROOT=$(cd $(dirname $0) && pwd)
SHARE_LIBS=$(cd ${SCRIPT_ROOT}/../share_libs; pwd)

if [ -f ${SHARE_LIBS}/shell/color.sh ]; then
    . ${SHARE_LIBS}/shell/color.sh
fi

if [ -f ${SHARE_LIBS}/shell/log.sh ]; then
    . ${SHARE_LIBS}/shell/log.sh
fi

#############
# Help Func #
#############

# help msg
usageMsg() {
cat << HELP

build_owner_docker.sh -- Build a new docker image with host local enviroment.

Usage:
  build_owner_docker.sh [-h|--help] [-t|--target-image TARGET_IMAGE_NAME] <-s|--source-image SOURCE_IMAGE_NAME:tag>

Options:
  -h, --help                Show this help message.

Note:
  You are expected to download $(echo -ne '\033[31mharbor.senseauto.com/senseauto/developer:latest\033[0m') before
  running this script.Please refer to the following steps which need root authority.

  0.login root
    $(echo -ne "\033[34m# sudo su\033[0m")
  1.edit $(echo -ne '\033[31m"/etc/hosts"\033[0m') and add a line: $(echo -ne '\033[31m"10.53.7.11 harbor.senseauto.com"\033[0m')
  2.edit $(echo -ne '\033[31m"/etc/docker/daemon.json"\033[0m') and add an item: $(echo -ne '\033[31m"insecure-registries": ["harbor.senseauto.com"]\033[0m')
  3.restart docker
    $(echo -ne "\033[34m# systemctl daemon-reload && systemctl restart docker\033[0m")
  4.logout root
    $(echo -ne "\033[34m# exit\033[0m")
  5.docker login
    $(echo -ne "\033[34m\$ docker login -u developer -p Goodsense@2019 harbor.senseauto.com\033[0m")
  6.pull image
    $(echo -ne "\033[34m\$ docker pull harbor.senseauto.com/senseauto/developer:latest\033[0m")
  7.run script
    $(echo -ne "\033[34m\$ bash build_owner_docker.sh --source-image='harbor.senseauto.com/senseauto/developer:latest' --target-image=my_image:from_developer\033[0m")

HELP
return 0
}

##################
# Parse argument #
##################

TEMP=$(getopt -o hs:t: --long help,source-image:,target-image: -n 'build_owner_docker.sh' -- "$@")
if [ $? != 0 ];then echoE "Fail parse arguments"; exit 1; fi

eval set -- "${TEMP}"

## Default setting
TARGET_IMAGE_NAME=""
SOURCE_IMAGE_NAME=harbor.senseauto.com/senseauto/developer:latest

while true; do
    case $1 in
        -h|--help)
            usageMsg && exit 0 ;;
        -s|--source-image)
            SOURCE_IMAGE_NAME=${2}; shift 2; ;;
        -t|--target-image)
            TARGET_IMAGE_NAME=${2}; shift 2; ;;
        --)
            shift ;
            if [[ ${#@} > 0 ]];
            then
                echoE "Unrecognized arguments: ${@}"
                exit 1
            fi
            break ;;
        *)
            echoE "Internal error"; exit 1 ;;
    esac
done

# set image name
HOSTNAME=$(hostname)
DEFAULT_TARGET_IMAGE_NAME="senseauto-dev:${HOSTNAME:-latest}"
if [ "x$TARGET_IMAGE_NAME" = "x" ]; then
    echoW "no image name input and will use default: ${DEFAULT_TARGET_IMAGE_NAME}"
    TARGET_IMAGE_NAME=${DEFAULT_TARGET_IMAGE_NAME}
fi
echoI "source image: ${SOURCE_IMAGE_NAME}"
echoI "target image: ${TARGET_IMAGE_NAME}"

#########
# Check #
#########
echoI "start checking ..."
# check binary
if [ ! -x $(which docker) ]; then
    echoE "can not found command: docker"
    exit 1
fi
# check image
if [ "x$(docker images -q ${SOURCE_IMAGE_NAME} 2> /dev/null)" == "x" ]; then
    echoE "can not found source image: ${SOURCE_IMAGE_NAME}"
    exit 1
fi
if [ "x$(docker images -q ${TARGET_IMAGE_NAME} 2> /dev/null)" != "x" ]; then
    echoE "found existed target image: ${TARGET_IMAGE_NAME}"
    exit 1
fi
# check file
if [ ! -f "${SCRIPT_ROOT}/owner-Dockerfile" ]; then
    echoE "can not found file: ${SCRIPT_ROOT}/owner-Dockerfile"
    exit 1
fi
echoI "checking passed"


###########
# Prepare #
###########

TMP_DIR=${SCRIPT_ROOT}/tmp/data
mkdir -p ${TMP_DIR}

# files for copy
files=(
${HOME}/.gitconfig # git config and git lfs credentials
${HOME}/.git-credentials
${HOME}/.tmux.conf
)
for file in ${files[@]}; do
    if [ -d "${file}" ]; then
        rm -rf ${file}
    fi
    if [ ! -f "${file}" ]; then
        touch ${file}
    fi
    cp -rfL ${file} $TMP_DIR/
done

# prepare ssh
if [ -d "${HOME}/.ssh" ]; then
    cp -rfL ${HOME}/.ssh $TMP_DIR/
else
    mkdir -p $TMP_DIR/.ssh
fi

# prepare aws
if [ -d "${HOME}/.aws" ]; then
    cp -rfL ${HOME}/.aws $TMP_DIR/
else
    mkdir -p $TMP_DIR/.aws
fi

######################
# Build docker image #
######################
echoI "Building image ..."

set -x

cd ${SCRIPT_ROOT}

if [ $(id -u) -ne 0 ]; then
    docker build -t ${TARGET_IMAGE_NAME} \
        -f owner-Dockerfile \
        --build-arg source_image_name=${SOURCE_IMAGE_NAME} \
        --build-arg userid=$(id -u) \
        --build-arg groupid=$(id -g) .
else
    echoW "You are already super user, will not built with userid and groupid"
    docker build -t ${TARGET_IMAGE_NAME} -f owner-Dockerfile \
        --build-arg source_image_name=${SOURCE_IMAGE_NAME} .
fi
set +x
echoI "Building successfully!"

cd - > /dev/null 2>&1

#########
# Clear #
#########

rm -rf $TMP_DIR
