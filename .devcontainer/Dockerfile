FROM ros:jazzy

ARG USERNAME=dev
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Delete user if it exists in base image (likely 'ubuntu' in noble)
RUN if id -u $USER_UID ; then userdel `id -un $USER_UID`; fi

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    #
    # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Install handy tools & potential GUI requirements (x11-apps for testing)
RUN apt-get update && apt-get install -y \
    git \
    bash-completion \
    python3-pip \
    python3-argcomplete \
    python3-tk \
    x11-apps \
    && rm -rf /var/lib/apt/lists/*

# Install RMF dependencies and Web development tools
RUN apt-get update && apt-get install -y \
    curl \
    python3-venv \
    python3-pip \
    pipenv \
    python3-opencv \
    ros-jazzy-rmf-visualization \
    ros-jazzy-rmf-task-ros2 \
    ros-jazzy-rmf-traffic-ros2 \
    ros-jazzy-rqt-graph \
    ros-jazzy-rmf-fleet-msgs \
    ros-jazzy-rmf-traffic-editor \
    && rm -rf /var/lib/apt/lists/*

# Install Node.js 20 and pnpm
RUN curl -fsSL https://deb.nodesource.com/setup_20.x | bash - \
    && apt-get install -y nodejs \
    && npm install -g pnpm \
    && rm -rf /var/lib/apt/lists/*

# Install rmf web dependencies
# WORKDIR /home/dev/ros2_ws/src/rmf-web
# RUN pnpm install

# Expose rmf-web default port
EXPOSE 5173

ENV SHELL /bin/bash

# Source ROS setup in .bashrc for the user
RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/$USERNAME/.bashrc

USER $USERNAME
CMD ["/bin/bash"]
