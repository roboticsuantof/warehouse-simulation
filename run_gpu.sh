#!/bin/bash
# Script para ejecutar la simulación de Gazebo con aceleración por hardware (NVIDIA GPU).
# REQUISITO: Tener instalado el NVIDIA Container Toolkit en la máquina host.

# Nombre de la imagen de Docker que construiste
IMAGE_NAME="sim_gazebo"

# Comprueba si la imagen existe
if ! docker image inspect "$IMAGE_NAME" &> /dev/null; then
    echo "Error: La imagen de Docker '$IMAGE_NAME' no se ha encontrado."
    echo "Por favor, primero construye la imagen con: docker build -t $IMAGE_NAME ."
    exit 1
fi

echo "Lanzando simulación de Gazebo con aceleración por hardware (NVIDIA GPU)..."

# Permite al contenedor conectarse al servidor X del host
xhost +local:docker

# Ejecuta el contenedor
docker run -it --rm \
  --gpus all \
  -v /dev/dri:/dev/dri \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v $(pwd)/ignition_cache:/home/simuser/.ignition \
  "$IMAGE_NAME"

# Revoca el permiso cuando el contenedor se detiene
xhost -local:docker
