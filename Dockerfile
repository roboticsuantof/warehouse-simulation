# 1. Entorno Base
FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# 2. Instalar dependencias básicas, Gazebo y ROS 2
RUN apt-get update && apt-get install -y \
    lsb-release gnupg curl wget \
    libgl1-mesa-dri libgl1-mesa-glx libglapi-mesa

# --- Instalar Gazebo ---
RUN curl -sSL https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list

# --- Instalar ROS 2 ---
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list

# --- Instalar paquetes de ROS y Gazebo ---
RUN apt-get update && apt-get install -y \
    ignition-fortress \
    ros-humble-desktop \
    ros-dev-tools \
    ros-humble-ros-gz \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-gz-ros2-control && \
    rm -rf /var/lib/apt/lists/*

# WORKAROUND para paquetes de ROS rotos que instalan en /local
RUN if [ -d /opt/ros/humble/local ]; then cp -a /opt/ros/humble/local/* /opt/ros/humble/ && rm -rf /opt/ros/humble/local; fi

# 3. Crear usuario y copiar archivos
RUN useradd -ms /bin/bash simuser
USER simuser
WORKDIR /home/simuser/app

COPY --chown=simuser:simuser . .

# 4. Construir el Workspace de ROS
RUN source /opt/ros/humble/setup.bash && \
    colcon build

# 5. Configurar entorno de ejecución
ENV DISPLAY=:0
ENV IGN_GAZEBO_RESOURCE_PATH=/home/simuser/app/models:/home/simuser/app/final_world

# 6. Comando de ejecución
CMD ["bash", "-c", "source /opt/ros/humble/setup.bash && source install/local_setup.bash && ros2 launch rb_theron_description_fortress spawn_robot.launch.py"]
