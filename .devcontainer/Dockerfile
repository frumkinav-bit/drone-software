# Берём готовый образ: Ubuntu 24.04 + ROS 2 Jazzy с графическими инструментами
FROM osrf/ros:jazzy-desktop

# Доустанавливаем нужные программы (apt-get — установщик Ubuntu)
# -y: отвечать «да» на вопросы; --no-install-recommends: не ставить лишнее
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-colcon-common-extensions \
    git \
    && rm -rf /var/lib/apt/lists/*
# последняя строка удаляет временные файлы, чтобы образ был меньше
