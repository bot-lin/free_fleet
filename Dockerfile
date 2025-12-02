FROM osrf/ros:jazzy-desktop-full

# 设置非交互式前端，防止安装过程卡住
ENV DEBIAN_FRONTEND=noninteractive

# 替换 ROS 2 源为清华源 (针对中国大陆用户优化网络连接)
# 这一步是为了解决 packages.ros.org 连接超时或被重置的问题
RUN sed -i 's#http://packages.ros.org/ros2/ubuntu#https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu#g' /etc/apt/sources.list.d/ros2.list || true

# 更新源并安装构建工具和请求的依赖
# 添加 --fix-missing 尝试修复下载失败的包
RUN apt-get update && apt-get install -y --fix-missing \
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

# 设置 bash 环境自动 source ROS (已注释)
# RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc

# 保持容器运行的默认命令（可被覆盖）
CMD ["bash"]
