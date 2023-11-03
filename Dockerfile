FROM ros:humble

ENV ROS_DISTRO humble

# install ros package
RUN apt-get update && apt-get install -y \
    python3-pip \
    curl \      
    && rm -rf /var/lib/apt/lists/*

# sourcing ROS
RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc
RUN echo "source install/local_setup.bash" >> /home/$USERNAME/.bashrc

WORKDIR /workspaces/cannon


COPY src src
COPY ros2_entrypoint.sh ros2_entrypoint.sh

#RUN . /opt/ros/humble/setup.bash && colcon build
RUN colcon build

# download rosbridge for Foxglove monitoring
RUN apt update && apt-get install -y ros-humble-rosbridge-suite ros-humble-rosbag2-storage-mcap ros-$ROS_DISTRO-foxglove-bridge

# ----------------FIX---------------------
# TODO: fix for production!
RUN pip install citros==1.2.52
# RUN pip install --no-cache-dir --upgrade pip \
#     && pip install --no-cache-dir citros
# ----------------FIX---------------------

RUN chmod +x ros2_entrypoint.sh
ENTRYPOINT ["/workspaces/cannon/ros2_entrypoint.sh"]

CMD ["bash"]