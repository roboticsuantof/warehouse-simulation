FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

# Instalar dependencias básicas y Gazebo Fortress en una sola capa
RUN apt-get update && \
    apt-get install -y lsb-release gnupg curl && \
    curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
        -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
        http://packages.osrfoundation.org/gazebo/ubuntu-stable \
        $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list && \
    apt-get update && \
    apt-get install -y ignition-fortress && \
    rm -rf /var/lib/apt/lists/*
RUN apt-get update && \
    apt-get install -y libgl1-mesa-dri libgl1-mesa-glx libglapi-mesa && \
    rm -rf /var/lib/apt/lists/*
# Crear usuario no-root
RUN useradd -ms /bin/bash simuser

USER simuser
WORKDIR /home/simuser/app

# Copiar el código del repositorio al contenedor
COPY --chown=simuser:simuser . .

# Variables de entorno para GUI y recursos
ENV DISPLAY=:0
ENV PATH=/home/simuser/.local/bin:$PATH
# Ajusta estas rutas según dónde estén tus modelos y texturas en el repo
ENV IGN_GAZEBO_RESOURCE_PATH=/home/simuser/app/models
ENV IGN_GAZEBO_PLUGIN_PATH=/home/simuser/app/plugins

# Comando por defecto al iniciar el contenedor
CMD ["ign", "gazebo", "final_world/industrial-warehouse.sdf"]
