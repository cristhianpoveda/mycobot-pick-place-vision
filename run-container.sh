#!/bin/bash

NAME=mycobot/vision
TAG=1.0
VOLUME=pick-place-pkgs

mkdir -p ${VOLUME}

docker volume create --driver local \
    --opt type="none" \
    --opt device="${PWD}/${VOLUME}/" \
    --opt o="bind" \
    "${VOLUME}"

xhost +
docker run \
    --name=perception \
    --net=host \
    --ipc=host \
    --env DISPLAY=${DISPLAY} \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="/dev:/dev" \
    --privileged \
    -it \
    --rm \
    --volume="${VOLUME}:/home/mycobot/catkin_ws/src/:rw" \
    "${NAME}:${TAG}"

exit 0