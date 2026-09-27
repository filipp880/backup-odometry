FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# Системные пакеты, ROS 2 зависимости и библиотеки анализа
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    python3-colcon-common-extensions \
    python3-pip \
    ros-humble-rosbag2 \
    ros-humble-rosbag2-storage-mcap \
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs \
    ros-humble-nav-msgs \
    ros-humble-diagnostic-msgs \
    ros-humble-std-msgs \
    ros-humble-rosidl-default-generators \
    ros-humble-rosidl-default-runtime \
    && pip3 install --no-cache-dir rosbags numpy \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws

# Копируем исходники
COPY tram_vehicle_msgs /ws/src/tram_vehicle_msgs
COPY tram_odometry /ws/src/tram_odometry

# Сборка пакетов
RUN . /opt/ros/humble/setup.sh && \
    colcon build \
    --packages-select tram_vehicle_msgs tram_odometry \
    --cmake-args -DCMAKE_BUILD_TYPE=Release

# Симлинк для доступа к ML-моделям
RUN ln -s /ws/src/tram_odometry/models /ws/models

# Скрипты запуска и оценки
COPY entrypoint.sh /entrypoint.sh
COPY evaluate.py /ws/evaluate.py
COPY run_benchmark.sh /ws/run_benchmark.sh
RUN chmod +x /entrypoint.sh /ws/evaluate.py /ws/run_benchmark.sh

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /ws/install/setup.bash" >> /root/.bashrc

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
