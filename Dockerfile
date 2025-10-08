# 使用官方的 Ubuntu 22.04 (Jammy) 作为基础镜像
FROM ubuntu:22.04

# 设置环境变量，避免安装过程中的交互式提问
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Shanghai

# 安装基础软件和 ROS 2 GPG 密钥
RUN apt-get update && apt-get install -y \
    curl \
    gnupg \
    lsb-release \
    locales \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# 设置系统语言环境为英文UTF-8，避免ROS出现未知问题
RUN locale-gen en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG en_US.UTF-8

# 添加 ROS 2 Humble (jammy) 的软件源
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 更新软件源并安装 ROS 2 Humble 桌面版和项目所有依赖
RUN apt-get update && apt-get install -y \
    ros-humble-desktop \
    ros-humble-moveit \
    ros-humble-example-interfaces \
    ros-dev-tools

# 设置容器内的工作目录
WORKDIR /root/ros2_ws

# 设置容器启动时默认执行的命令
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && exec /bin/bash"]
