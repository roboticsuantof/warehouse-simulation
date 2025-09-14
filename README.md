# Warehouse Simulation with Gazebo Fortress

This repository contains a simulation of an industrial warehouse environment using Gazebo Fortress, packaged in a Docker container.

## Prerequisites

### Required Software:

1.  **Docker:** Essential for building and running the container image.
    *   For installation instructions, see the [official Docker documentation](httpss://docs.docker.com/engine/install/).

2.  **NVIDIA Container Toolkit (Optional):** Required only if you want to run the simulation with hardware acceleration (NVIDIA GPU).
    *   For installation, follow the instructions in the [NVIDIA documentation](httpss://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html).

## Installation

Clone the repository to your local machine:

```bash
git clone httpss://github.com/roboticsuantof/warehouse-simulation.git
cd warehouse-simulation
```

## Usage

The project includes two scripts to run the simulation, depending on whether you want to use the CPU or an NVIDIA GPU for rendering.

### Running with CPU

This mode uses software rendering and does not require an NVIDIA GPU.

1.  **Build the image and run the simulation:**
    The `run_cpu.sh` script will handle building the Docker image and launching the container.

    ```bash
    bash run_cpu.sh
    ```

### Running with GPU Acceleration (NVIDIA)

This mode leverages an NVIDIA GPU for rendering, which generally results in better performance.

1.  **Build the Docker image:**
    First, build the image manually.

    ```bash
    docker build -t sim_gazebo .
    ```

2.  **Run the simulation:**
    Then, use the `run_gpu.sh` script to start the container with GPU support.

    ```bash
    bash run_gpu.sh
    ```

When you run either of the scripts, the Gazebo Fortress simulation window should appear after a moment, loading the warehouse world.

The first time the simulation is started, it takes a while, but the rest of the executions will take less time due to caching.
