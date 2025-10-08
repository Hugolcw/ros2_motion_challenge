# ROS 2 机械臂运动控制挑战 - 最终成果

本项目作为“梯队国庆项目考核”的一部分，成功实现了一个C++ ROS 2节点，用于控制一个六轴仿真机械臂运动到预设位置。

## 1. 项目方案概述

本项目基于官方提供的仿真功能包，独立编写了一个名为 `arm_controller` 的C++功能包。

在开发过程中，由于原始项目推荐的 `example_interfaces` 依赖包在多个测试环境（包括本地Ubuntu 24.04/Jazzy、Docker Ubuntu 22.04/Humble）中均出现无法解决的底层环境问题，本项目采纳了原始 `README.md` 中【技术要求】部分明确允许的替代方案。

最终方案未使用依赖字符串的服务，而是通过 **`std_srvs/srv/Trigger`** 这种更为稳定和基础的服务类型，创建了三个独立的ROS 2服务：
* `/move_to_home`
* `/move_to_pose1`
* `/move_to_pose2`

用户通过调用这些特定的服务来“触发”机械臂移动到对应的、在程序中硬编码的关节角度位置。

## 2. 环境配置

本项目最终在一个与项目要求完全匹配的、纯净的Docker环境中成功编译和运行。
* **主机操作系统**: 任意支持Docker的Linux系统 (本项目在Ubuntu 24.04上完成)
* **Docker镜像**: `osrf/ros:humble-desktop` (Ubuntu 22.04 + ROS 2 Humble)
* **核心依赖**: Docker, Git

## 3. 编译与运行指南

### 第一步：克隆仓库

```bash
git clone [https://github.com/Hugolcw/ros2_motion_challenge.git](https://github.com/Hugolcw/ros2_motion_challenge.git)
mv ros2_motion_challenge motion_challenge_ws
```

### 第二步：启动Docker容器
在 motion_challenge_ws 的上级目录中，打开终端，执行以下命令来启动一个配置好ROS 2 Humble的容器，并将项目工作区挂载进去。

```bash
# 允许图形界面连接
xhost +

# 启动容器
docker run -it \
  --name humble_dev \
  -v ~/motion_challenge_ws:/root/motion_challenge_ws \
  --net=host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  osrf/ros:humble-desktop
```
执行后，您将进入容器的命令行环境（提示符为 root@...)

### 第三步：在容器内安装依赖并编译
在容器终端中，执行一下命令来安装所有必要的依赖并编译工作区。

1. 更新系统并安装基础工具
```bash
apt-get update && apt-get install -y python3-rosdep git
```

2. 初始化并更新rosdep
```bash
rosdep init
rosdep update
# (如果 rosdep init 提示“already exists”，这是正常的，可以忽略)
```

3. 进入工作区并安装项目所有依赖（此步会花费几分钟）
```bash
cd /root/motion_challenge_ws
rosdep install --from-paths src --ignore-src -r -y
```
4. 最终编译
```bash
colcon build
```

### 第四步：运行与测试
编译成功后，需要打开三个终端来运行和测试。所有操作均在容器内完成。

终端1（启动仿真环境）

在刚刚那个编译成功的容器终端中，执行：
```bash
source /root/motion_challenge_ws/install/setup.bash
ros2 launch simple_robot_bringup demo.launch.py
```
你会看到RViz可视化窗口弹出。

终端2（运行您的控制节点）

在本地电脑上打开一个新的终端,使用 docker exec 命令进入正在运行的容器：
```bash
docker exec -it humble_dev bash
```
进入容器后，运行节点：
```bash
cd /root/motion_challenge_ws
source install/setup.bash
ros2 run arm_controller position_controller_node
```

终端3（调用服务）

在本地电脑上再打开一个新的终端,同样的，进入容器：
```bash
docker exec -it humble_dev bash
```

进入容器后，调用服务来控制机械臂：
```bash
cd /root/motion_challenge_ws
source install/setup.bash

# 命令机械臂去 "home" 位置
ros2 service call /move_to_home std_srvs/srv/Trigger

# 命令机械臂去 "pose1" 位置
ros2 service call /move_to_pose1 std_srvs/srv/Trigger

# 命令机械臂去 "pose2" 位置
ros2 service call /move_to_pose2 std_srvs/srv/Trigger
```

## 4.API使用简述
本节点的核心功能是通过 moveit::planning_interface::MoveGroupInterface 类来实现的。主要遵循了”设置目标 -> 规划 -> 执行“的流程：

1. setJointValueTarget()：在收到服务请求后，将该请求对应的硬编码关节角度数组（一个 std::vector<double>） 传递给此函数，用于告知 Moveit 我们的运动目标。

2. plan()：调用 Moveit 的运动规划器，根据设定的目标计算出一条无碰撞的运动轨迹。此函数的返回值用于判断规划是否成功。

3. execute()：在确认 plan() 成功后，调用此函数，并将规划结果作为参数传入。该函数会将轨迹发送给机器人控制器，驱动仿真机器人完成物理运动。

## 5.思考题解答

***问题：在你的实现中，机械臂是沿着直线运动到目标点的吗？如果不是，请解释为什么。并说明 Moveit 中的”关节空间规划“和”笛卡尔空间规划“有何不同。***

**回答：在我的实现中，机械臂的末端不是沿着一条直线运动到目标点的，而是划出一条平滑的曲线。这是因为我们使用 setJointValueTarget() 函数来指定目标，这触发的是 Moveit 的关节空间规划。**

*关节空间规划：它的目标是为机械臂的每一个关节找到一条从初始角度到目标角度的平滑、最短（在关节空间意义上）的路径。规划器主要关心的是如何让每个关节平稳、高效地到达终点角度，而机械臂末端在三维空间中最终走出的轨迹，是所有关节同时运动产生的自然结果，通常是一条曲线。*

*笛卡尔空间规划：它的目标是让机械臂的末端执行器在三维空间中沿着一条预先指定的几何路径（例如一条直线，一段圆弧）运动。规划器会在这条路径上取一系列的路径点，然后通过逆运动学反复求解，计算出在每个路径点上所有关节应该处于的角度，最终形成一条能让末端走出指定路径的轨迹。*

