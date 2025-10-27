FROM osrf/ros:humble-desktop

ARG USERNAME=devuser
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ENV SHELL /bin/bash

RUN apt-get update && apt-get upgrade -y && \
    # catkin install
    # python3-catkin-pkg \
    apt-get install -y --no-install-recommends \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-control-msgs \
    ros-humble-joint-state-publisher-gui \
    ros-humble-xacro \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    sudo \
    git \
    python3-pip \
    python3-colcon-common-extensions \
    && \
    groupadd --gid $USER_GID $USERNAME && \
    useradd --uid $USER_UID --gid $USER_GID -m $USERNAME && \
    echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME && \
    rm -rf /var/lib/apt/lists/*

USER $USERNAME

USER devuser
WORKDIR /home/devuser/projects/2025-2026/ws/src
RUN  git clone -b humble https://github.com/ros-controls/ros2_control_demos.git

RUN echo "source /opt/ros/humble/setup.bash" >> /home/${USERNAME}/.bashrc && \
    echo "source /home/${USERNAME}/ws/install/setup.bash" >> /home/${USERNAME}/.bashrc

CMD ["/bin/bash"]