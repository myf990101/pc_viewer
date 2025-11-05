# 基础镜像
FROM ubuntu:20.04

# 防止交互式提示
ENV DEBIAN_FRONTEND=noninteractive

# 安装必要依赖和 ROS Noetic
RUN apt-get update && apt-get install -y \
    curl gnupg2 lsb-release sudo \
    && sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
    && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - \
    && apt-get update && apt-get install -y \
       ros-noetic-desktop-full \
       python3-rosdep python3-pip python3-rosinstall python3-rosinstall-generator python3-wstool build-essential \
    && rosdep init || true \
    && rosdep update || true \
    && echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc \
    && apt-get clean && rm -rf /var/lib/apt/lists/*
    
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc

# 默认启动 bash
CMD ["bash"]
