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

# --- Switch to the new user ---
USER $USERNAME
WORKDIR /home/$USERNAME/ros2_ws

# 4. Copy the local workspace source code into the container
COPY --chown=devuser:devuser ./ws/src ./src

# 5. Builds the workspace
SHELL ["/bin/bash", "-c"] 
RUN . /opt/ros/humble/setup.bash && \
    colcon build --symlink-install

# 6. Configures the entrypoint
RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc && \
    echo "source /home/$USERNAME/ros2_ws/install/setup.bash" >> /home/$USERNAME/.bashrc