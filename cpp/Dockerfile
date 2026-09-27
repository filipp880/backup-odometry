FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# Установка необходимых системных пакетов и зависимостей ROS 2
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    python3-colcon-common-extensions \
    ros-humble-rosbag2 \
    ros-humble-rosbag2-storage-mcap \
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs \
    ros-humble-nav-msgs \
    ros-humble-diagnostic-msgs \
    ros-humble-std-msgs \
    ros-humble-rosidl-default-generators \
    ros-humble-rosidl-default-runtime \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws

# Копируем исходный код пакетов
COPY tram_vehicle_msgs /ws/src/tram_vehicle_msgs
COPY tram_odometry /ws/src/tram_odometry

# Сборка пакетов
RUN . /opt/ros/humble/setup.sh && \
    colcon build \
    --packages-select tram_vehicle_msgs tram_odometry \
    --cmake-args -DCMAKE_BUILD_TYPE=Release

# ВАЖНО: Делаем симлинк на папку models в рабочей директории /ws,
# чтобы относительный путь 'models/ml_model.yaml' корректно открывался
RUN ln -s /ws/src/tram_odometry/models /ws/models

# Настройка Entrypoint
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /ws/install/setup.bash" >> /root/.bashrc

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
