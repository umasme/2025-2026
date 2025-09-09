# If you have any question you can contact any of the software subteam leads.

# 1. Starts from the official ROS 2 Humble base image which is the LTS version
FROM osrf/ros:humble-desktop

# 2. Creates a non-root user
ARG USERNAME=devuser
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# 3. Installs the dependencies
RUN apt-get update && apt-get install -y \
    git \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

USER $USERNAME
WORKDIR /home/$USERNAME/ros2_ws

# 4. Clones the repository and structure the workspace
RUN git clone --depth 1 https://github.com/umasme/2025-2026.git temp_repo \
    && mv temp_repo/ws/src . \
    && rm -rf temp_repo

# 5. Builds the workspace
RUN . /opt/ros/humble/setup.bash && \
    colcon build --symlink-install

# 6. Configures the entrypoint
RUN echo "source /home/$USERNAME/ros2_ws/install/setup.bash" >> /home/$USERNAME/.bashrc