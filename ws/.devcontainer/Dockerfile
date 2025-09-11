
FROM osrf/ros:humble-desktop

ARG USERNAME=devuser
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ENV SHELL /bin/bash

RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y --no-install-recommends \
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

CMD ["/bin/bash"]