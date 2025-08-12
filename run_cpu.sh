#!/bin/bash

# Script para ejecutar la simulación de Gazebo en un contenedor Docker usando rendering por software.
# Persiste los cachés de Ignition en un directorio local para acelerar los lanzamientos posteriores.

# --- Configuración ---
IMAGE_NAME="gazebo-fortress-sim:latest"
CACHE_DIR="$(pwd)/ignition_cache"
WORLD_FILE="/workspace/final_world/industrial-warehouse.sdf"

# --- Lógica del Script ---

# 1. Crear directorios de caché locales si no existen
echo "Comprobando los directorios de caché en ${CACHE_DIR}..."
mkdir -p "${CACHE_DIR}/fuel"
mkdir -p "${CACHE_DIR}/gazebo"
mkdir -p "${CACHE_DIR}/gui"
mkdir -p "${CACHE_DIR}/rendering"
echo "Directorios de caché listos."

# 2. Construir la imagen de Docker
echo "Construyendo la imagen de Docker: ${IMAGE_NAME}..."
docker build -t "${IMAGE_NAME}" .
echo "Construcción completada."

# 3. Ejecutar la simulación
echo "Lanzando Gazebo Fortress (modo CPU)..."
echo "La ventana de la GUI puede tardar un momento en aparecer."

docker run -it --rm \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "$(pwd)":/workspace \
  -v "${CACHE_DIR}/fuel":/.ignition/fuel \
  -v "${CACHE_DIR}/gazebo":/.ignition/gazebo \
  -v "${CACHE_DIR}/gui":/.ignition/gui \
  -v "${CACHE_DIR}/rendering":/.ignition/rendering \
  "${IMAGE_NAME}" ign gazebo "${WORLD_FILE}"

echo "Simulación cerrada."
