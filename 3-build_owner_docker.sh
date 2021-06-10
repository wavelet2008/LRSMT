#!/bin/bash
# Copyright (C) 2019 by SenseTime Group Limited. All rights reserved.
# This script will build a new image from base image specified by args '--source-image'.
# In the container created by new image, you will have same permssion with local host user.
# And will copy some config files, such as: ~/.gitconfig, ~/.ssh, into new image.

set -e

SCRIPT_VERSION="1.0.0"
SCRIPT_ROOT=$(cd $(dirname $0) && pwd)
SHARE_LIBS=$(cd ${SCRIPT_ROOT}/../share_libs; pwd)

# set image name
SOURCE_IMAGE_NAME=uwsbel/projectchrono:latest
HOSTNAME=$(hostname)
TARGET_IMAGE_NAME="roboterrain:${HOSTNAME:-latest}"
echo "source image: ${SOURCE_IMAGE_NAME}"
echo "target image: ${TARGET_IMAGE_NAME}"

if [ "x$(docker images -q ${TARGET_IMAGE_NAME} 2> /dev/null)" != "x" ]; then
    echo "found existed target image: ${TARGET_IMAGE_NAME}"
    exit 1
fi
if [ ! -f "${SCRIPT_ROOT}/owner-Dockerfile" ]; then
    echo "can not found file: ${SCRIPT_ROOT}/owner-Dockerfile"
    exit 1
fi
echo "checking passed"

TMP_DIR=${SCRIPT_ROOT}/tmp/data
mkdir -p ${TMP_DIR}

echo "Building image ..."

set -x

cd ${SCRIPT_ROOT}

if [ $(id -u) -ne 0 ]; then
    docker build -t ${TARGET_IMAGE_NAME} \
        -f owner-Dockerfile \
        --build-arg source_image_name=${SOURCE_IMAGE_NAME} \
        --build-arg userid=$(id -u) \
        --build-arg groupid=$(id -g) .
else
    echo "You are already super user, will not built with userid and groupid"
    docker build -t ${TARGET_IMAGE_NAME} -f owner-Dockerfile \
        --build-arg source_image_name=${SOURCE_IMAGE_NAME} .
fi
set +x
echo "Building successfully!"

cd - > /dev/null 2>&1

rm -rf ${SCRIPT_ROOT}/tmp
