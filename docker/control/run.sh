#!/usr/bin/env bash
set -e
# set -x   # раскомментируй для отладки — покажет, какую именно команду запускает скрипт

IMAGE_NAME="stingray"
CONTAINER_NAME="stingray"

# xhost +si:localuser:root


if [ "$(docker ps -a -q -f name=^/${CONTAINER_NAME}$)" ]; then
    echo "Container '$CONTAINER_NAME' already exists. Removing..."
    docker rm -f $CONTAINER_NAME
fi

ROS_DOMAIN_ID=1

# запускаем контейнер от текущего пользователя, чтобы файлы в bind-mount были твоими
docker run -it --rm \
  --privileged \
  --name "$CONTAINER_NAME" \
  --network host \
  -v "$(pwd)":/stingray \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /dev:/dev\
  -e DISPLAY="$DISPLAY" \
  -e ROS_DOMAIN_ID=$ROS_DOMAIN_ID \
  "$IMAGE_NAME"
