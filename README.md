# Simulación de Almacén con Gazebo Fortress

Este repositorio contiene una simulación de un entorno de almacén industrial utilizando Gazebo Fortress, empaquetada en un contenedor Docker.

## Requisitos Previos

### Software necesario:

1.  **Docker:** Esencial para construir y ejecutar la imagen del contenedor.
    *   Para instrucciones de instalación, consulte la [documentación oficial de Docker](https://docs.docker.com/engine/install/).

2.  **NVIDIA Container Toolkit (Opcional):** Requerido únicamente si desea ejecutar la simulación con aceleración por hardware (GPU NVIDIA).
    *   Para la instalación, siga las instrucciones en la [documentación de NVIDIA](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html).

## Instalación

Clonar el repositorio en la máquina local:

```bash
git clone https://github.com/roboticsuantof/warehouse-simulation.git
cd warehouse-simulation
```

## Uso

El proyecto incluye dos scripts para ejecutar la simulación, dependiendo de si desea utilizar la CPU o una GPU NVIDIA para el renderizado.

### Ejecución con CPU

Este modo utiliza renderizado por software y no requiere una GPU NVIDIA.

1.  **Construir la imagen y ejecutar la simulación:**
    El script `run_cpu.sh` se encargará de construir la imagen de Docker y lanzar el contenedor.

    ```bash
    bash run_cpu.sh
    ```

### Ejecución con Aceleración por GPU (NVIDIA)

Este modo aprovecha una GPU NVIDIA para el renderizado, lo que generalmente resulta en un mejor rendimiento.

1.  **Construir la imagen de Docker:**
    Primero, construya la imagen manualmente.

    ```bash
    docker build -t sim_gazebo .
    ```

2.  **Ejecutar la simulación:**
    Luego, utilice el script `run_gpu.sh` para iniciar el contenedor con soporte para GPU.

    ```bash
    bash run_gpu.sh
    ```

Al ejecutar cualquiera de los scripts, la ventana de la simulación de Gazebo Fortress debería aparecer después de un momento, cargando el mundo del almacén.

La primera vez que se inicia la simulacion, tarda un poco, pero el resto de ejecuciones tardaran menos debido al guardado en la caché
