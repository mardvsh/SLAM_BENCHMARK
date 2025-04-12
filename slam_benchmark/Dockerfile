# Базовый образ с ROS
FROM ros:noetic-ros-core

# Устанавливаем дополнительные пакеты для ROS и Gazebo
RUN apt-get update && apt-get install -y \
  ros-noetic-turtlebot3 \
  ros-noetic-gazebo-ros-pkgs \
  ros-noetic-map-server \
  ros-noetic-cartographer \
  ros-noetic-cartographer-ros \
  ros-noetic-hector-slam \
  python3-opencv \
  python3-pip \
  && rm -rf /var/lib


# Устанавливаем Python-библиотеки
RUN pip3 install matplotlib psutil numpy opencv-python scikit-image

# Рабочая директория
WORKDIR /root/catkin_ws

# Копируем все исходники в рабочую директорию контейнера
COPY . /root/catkin_ws/src/

# Сборка catkin workspace
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# Открытие bash с необходимыми настройками
CMD ["/bin/bash", "-c", "source /opt/ros/noetic/setup.bash && source /root/catkin_ws/devel/setup.bash && roslaunch turtlebot3_gazebo turtlebot3_world.launch"]
