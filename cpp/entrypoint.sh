#!/bin/bash
set -e

# Подключение базового ROS 2 и собранного воркспейса
source /opt/ros/humble/setup.bash
if [ -f "/ws/install/setup.bash" ]; then
    source /ws/install/setup.bash
fi

exec "$@"
