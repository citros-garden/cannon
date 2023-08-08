FROM ros:humble

ENV ROS_DISTRO humble

# install ros package
RUN apt-get update && apt-get install -y \
    python3-pip \
    curl \      
    && rm -rf /var/lib/apt/lists/*

# RUN sudo apt-get update && apt-get install -y \
#     ros-$ROS_DISTRO-rosbag2-storage-mcap \
#     ros-$ROS_DISTRO-rosbag2 \
#     ros-$ROS_DISTRO-ros-base \
#     ros-$ROS_DISTRO-ros2bag \
#     ros-$ROS_DISTRO-rosbag2-transport \
#     && rm -rf /var/lib/apt/lists/*
# RUN pip install setuptools==58.2.0

# sourcing ROS
RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc
RUN echo "source install/local_setup.bash" >> /home/$USERNAME/.bashrc

WORKDIR /workspaces/cannon


COPY src src
COPY ros2_entrypoint.sh ros2_entrypoint.sh

#RUN . /opt/ros/humble/setup.bash && colcon build
RUN colcon build

# download rosbridge for Foxglove monitoring
RUN apt update && apt-get install -y ros-humble-rosbridge-suite

# ----------------FIX--------------------
# TODO: fix for production!
# RUN pip install citros
COPY .citros .citros
COPY utils utils
RUN pip install utils/citros_cli
# ----------------FIX--------------------

RUN chmod +x ros2_entrypoint.sh
ENTRYPOINT ["/workspaces/cannon/ros2_entrypoint.sh"]

CMD ["bash"]