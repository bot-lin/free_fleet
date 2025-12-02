FROM ros:jazzy

# 设置非交互式前端，防止安装过程卡住
ENV DEBIAN_FRONTEND=noninteractive

# 更新源并安装构建工具和请求的依赖
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    git \
    ros-jazzy-rmf-dev \
    && rm -rf /var/lib/apt/lists/*

# 初始化 rosdep (如果需要要在容器内安装其他依赖)
RUN rosdep init || echo "rosdep already initialized" && \
    rosdep update

# 创建工作空间目录
WORKDIR /data/free_fleet_ws

# 设置 bash 环境自动 source ROS
# RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc

# 保持容器运行的默认命令（可被覆盖）
CMD ["bash"]

