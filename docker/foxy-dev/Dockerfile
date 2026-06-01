FROM ros:foxy-ros-base-focal

# 接続の調子がおかしいときにUbuntuミラーを日本側に変える
# RUN sed -i 's|http://archive.ubuntu.com/ubuntu|http://ftp.jaist.ac.jp/pub/Linux/ubuntu|g' /etc/apt/sources.list && \
#     sed -i 's|http://security.ubuntu.com/ubuntu|http://ftp.jaist.ac.jp/pub/Linux/ubuntu|g' /etc/apt/sources.list && \
#     sed -i 's|http://jp.archive.ubuntu.com/ubuntu|http://ftp.jaist.ac.jp/pub/Linux/ubuntu|g' /etc/apt/sources.list

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# 基本ツール
RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    vim \
    nano \
    wget \
    curl \
    build-essential \
    cmake \
    python3-pip \
    python3-usb \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-argcomplete \
    && rm -rf /var/lib/apt/lists/*

# GUIツール
RUN apt-get update && apt-get install -y --no-install-recommends \
    x11-apps \
    mesa-utils \
    libgl1-mesa-glx \
    libglib2.0-0 \
    gnupg2 \
    lsb-release \
    && rm -rf /var/lib/apt/lists/*

# USBデバイス関係
RUN apt-get update && apt-get install -y --no-install-recommends \
    usbutils \
    libusb-1.0-0-dev \
    python3-serial \
    rtklib \
    iputils-ping \
    netcat-openbsd \
    && rm -rf /var/lib/apt/lists/*

# ROS関係
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-foxy-image-pipeline \
    ros-foxy-depthai-ros \
    ros-foxy-rtabmap \
    ros-foxy-rtabmap-ros \
    ros-foxy-rviz2 \
    ros-foxy-rqt-image-view \
    ros-foxy-rqt \
    ros-foxy-tf-transformations \
    ros-foxy-robot-localization \
    ros-foxy-tf2-tools \
    ros-foxy-rtcm-msgs \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --upgrade pip
RUN python3 -m pip install --no-cache-dir \
    "setuptools<70" \
    "importlib-metadata>=4.6,<7" \
    wheel

# ODrive関係
RUN python3 -m pip install --upgrade odrive

# OAK-D / DepthAI Python API
RUN python3 -m pip install --upgrade pip && \
    python3 -m pip install --no-cache-dir \
    numpy==1.24.4 \
    opencv-python==4.13.0.92
# DepthAIはv3が最新だが、Foxyで動かすならv2のほうがいい説
RUN python3 -m pip install --upgrade pip && \
    python3 -m pip install --no-cache-dir \
    depthai==2.30.0.0

# IMU関係
RUN python3 -m pip install --no-cache-dir \
    modbus_tk \
    transforms3d

# フォーマッタ
RUN python3 -m pip install --no-cache-dir black==24.4.2

# rosdep 初期化はホストや既存状態によって失敗することがあるので、失敗しても継続
RUN rosdep init || true
RUN rosdep update || true

# ワークスペース
RUN mkdir -p /workspaces/patasmonkey_ws/src
RUN mkdir -p /workspaces/ros2_ws/src
WORKDIR /workspaces/patasmonkey_ws

# bash 起動時に ROS / Foxy用 install を source
RUN echo "source /opt/ros/foxy/setup.bash" >> /root/.bashrc
RUN echo "if [ -f /workspaces/ros2_ws/install_foxy/setup.bash ]; then source /workspaces/ros2_ws/install_foxy/setup.bash; fi" >> /root/.bashrc
RUN echo "if [ -f /workspaces/ros2_ws/install/setup.bash ]; then source /workspaces/ros2_ws/install/setup.bash; fi" >> /root/.bashrc
RUN echo "if [ -f /workspaces/patasmonkey_ws/install_foxy/setup.bash ]; then source /workspaces/patasmonkey_ws/install_foxy/setup.bash; fi" >> /root/.bashrc
RUN echo "if [ -f /workspaces/patasmonkey_ws/install/setup.bash ]; then source /workspaces/patasmonkey_ws/install/setup.bash; fi" >> /root/.bashrc

# Foxy用ビルドalias
# RUN echo "alias build_ros2_foxy='cd /workspaces/ros2_ws && colcon --log-base log_foxy build --symlink-install --build-base build_foxy --install-base install_foxy --packages-select hwt905_rs485_driver'" >> /root/.bashrc
# RUN echo "alias build_pm_foxy='cd /workspaces/patasmonkey_ws && colcon --log-base log_foxy build --symlink-install --build-base build_foxy --install-base install_foxy'" >> /root/.bashrc
# RUN echo "alias src_foxy='source /opt/ros/foxy/setup.bash && if [ -f /workspaces/ros2_ws/install_foxy/setup.bash ]; then source /workspaces/ros2_ws/install_foxy/setup.bash; fi && if [ -f /workspaces/patasmonkey_ws/install_foxy/setup.bash ]; then source /workspaces/patasmonkey_ws/install_foxy/setup.bash; fi'" >> /root/.bashrc

CMD ["/bin/bash"]