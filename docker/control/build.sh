#!/bin/bash
set -e

IMAGE_NAME=stingray

echo "[INFO] Сборка Docker-образа: $IMAGE_NAME"
docker build -t $IMAGE_NAME -f docker/control/Dockerfile .

echo "[INFO] Образ успешно собран."
