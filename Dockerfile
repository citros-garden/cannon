FROM althack/ros2:humble-dev 

# ** [Optional] Uncomment this section to install additional packages. **
#
# ENV DEBIAN_FRONTEND=noninteractive
# RUN apt-get update \
#    && apt-get -y install --no-install-recommends <your-package-list-here> \
#    #
#    # Clean up
#    && apt-get autoremove -y \
#    && apt-get clean -y \
#    && rm -rf /var/lib/apt/lists/*
# ENV DEBIAN_FRONTEND=dialog

# Set up auto-source of workspace for ros user
ARG WORKSPACE
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/ros/.bashrc

# download rosbridge for Foxglove monitoring
RUN apt update && apt-get install -y ros-humble-rosbridge-suite

RUN pip install citros

WORKDIR /workspaces/citros_cannon

COPY src src

RUN . /opt/ros/humble/setup.bash && colcon build

# sourcing ROS
RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc
RUN echo "source install/local_setup.bash" >> /home/$USERNAME/.bashrc

CMD ["bash"]