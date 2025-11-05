# 使用 ROS Noetic 官方镜像
FROM ros:noetic

# 设置非交互安装
ENV DEBIAN_FRONTEND=noninteractive

# 安装常用工具和 catkin
RUN apt update && \
    apt install -y \
    python3-catkin-tools \
    python3-rosdep \
    git \
    vim \
    tmux \
    build-essential \
    && rosdep init && rosdep update

# 自动 source ROS
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc

# 设置工作目录
WORKDIR /root/ws
