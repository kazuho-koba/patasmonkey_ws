FROM ros:foxy-ros-base-focal

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# 基本ツール
RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    vim \
    nano \
    wget \
    curl \
    gnupg2 \
    lsb-release \
    build-essential \
    cmake \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-argcomplete \
    python3-ament-package \
    x11-apps \
    mesa-utils \
    libgl1-mesa-glx \
    libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/*

# rosdep 初期化はホストや既存状態によって失敗することがあるので、失敗しても継続
RUN rosdep init || true
RUN rosdep update || true

# ワークスペース
RUN mkdir -p /workspaces/patasmonkey_ws/src
WORKDIR /workspaces/patasmonkey_ws

# bash 起動時に ROS を source
RUN echo "source /opt/ros/foxy/setup.bash" >> /root/.bashrc
RUN echo "if [ -f /workspaces/patasmonkey_ws/install/setup.bash ]; then source /workspaces/patasmonkey_ws/install/setup.bash; fi" >> /root/.bashrc

CMD ["/bin/bash"]
