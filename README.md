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
在 **motion_challenge_ws** 的上级目录中，打开终端，执行以下命令来启动一个配置好ROS 2 Humble的容器，并将项目工作区挂载进去。

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
***执行后，您将进入容器的命令行环境（提示符为 root@...）***

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
本节点的核心功能是通过 **` moveit::planning_interface::MoveGroupInterface`** 类来实现的。主要遵循了”设置目标 -> 规划 -> 执行“的流程：

1. **`setJointValueTarget()`**：在收到服务请求后，将该请求对应的硬编码关节角度数组（一个 **`std::vector<double>`**） 传递给此函数，用于告知 **`Moveit`** 我们的运动目标。

2. **`plan()`**：调用 **`Moveit`** 的运动规划器，根据设定的目标计算出一条无碰撞的运动轨迹。此函数的返回值用于判断规划是否成功。

3. **`execute()`**：在确认 **`plan()`** 成功后，调用此函数，并将规划结果作为参数传入。该函数会将轨迹发送给机器人控制器，驱动仿真机器人完成物理运动。

## 5.思考题解答

***问题：在你的实现中，机械臂是沿着直线运动到目标点的吗？如果不是，请解释为什么。并说明 Moveit 中的”关节空间规划“和”笛卡尔空间规划“有何不同。***

**回答：在我的实现中，机械臂的末端不是沿着一条直线运动到目标点的，而是划出一条平滑的曲线。这是因为我们使用 setJointValueTarget() 函数来指定目标，这触发的是 Moveit 的关节空间规划。**

*关节空间规划：它的目标是为机械臂的每一个关节找到一条从初始角度到目标角度的平滑、最短（在关节空间意义上）的路径。规划器主要关心的是如何让每个关节平稳、高效地到达终点角度，而机械臂末端在三维空间中最终走出的轨迹，是所有关节同时运动产生的自然结果，通常是一条曲线。*

*笛卡尔空间规划：它的目标是让机械臂的末端执行器在三维空间中沿着一条预先指定的几何路径（例如一条直线，一段圆弧）运动。规划器会在这条路径上取一系列的路径点，然后通过逆运动学反复求解，计算出在每个路径点上所有关节应该处于的角度，最终形成一条能让末端走出指定路径的轨迹。*

# Motion Challenge

## 项目简介

本仓库提供了一个已配置好的六轴机械臂仿真功能包,你们需要在此基础上编写C++节点来控制机械臂运动。

## 系统要求

- **操作系统**: Ubuntu 22.04
- **ROS版本**: ROS 2 Humble
- **依赖**: MoveIt 2

## 安装依赖

在开始之前,请确保已安装ROS 2 Humble和MoveIt 2:

```bash
# 安装ROS 2 Humble (如果尚未安装)
# 参考: https://docs.ros.org/en/humble/Installation.html or rosfish
# 缺啥包装啥

# 安装MoveIt 2
sudo apt update
sudo apt install ros-humble-moveit
sudo apt install ros-humble-moveit-resources
sudo apt install ros-humble-moveit-visual-tools
sudo apt install ros-humble-ros2-control
sudo apt install ros-humble-ros2-controllers
sudo apt install ros-humble-controller-manager
```

## 编译

```bash
cd ~/motion_challenge_ws
colcon build --symlink-install
source install/setup.bash
```

## 运行Demo

启动机械臂仿真环境:

```bash
ros2 launch simple_robot_bringup demo.launch.py
```

启动后,你将看到:
- **RViz**: 可视化界面,显示机械臂模型
- **MoveIt MotionPlanning插件**: 可以通过拖动交互标记(Interactive Markers)手动规划路径
- **move_group**: 后台运行的MoveIt运动规划服务

### 测试手动运动规划

1. 在RViz中,你会看到机械臂模型和橙色的交互机械臂
2. 控制橙色机械臂到目标位置
3. 点击"Plan"按钮生成运动轨迹
4. 点击"Execute"按钮执行规划的轨迹

## 项目结构

```
motion_challenge_ws/
├── src/
│   ├── simple_robot_description/      # 机械臂URDF模型描述
│   │   ├── urdf/                      # URDF文件
│   │   ├── meshes/                    # 3D网格文件
│   │   ├── launch/                    # 启动文件
│   │   └── rviz/                      # RViz配置
│   │
│   ├── simple_robot_moveit_config/    # MoveIt配置包
│   │   ├── config/                    # MoveIt配置文件
│   │   │   ├── simple_robot.srdf     # 语义描述文件
│   │   │   ├── joint_limits.yaml     # 关节限制
│   │   │   ├── kinematics.yaml       # 运动学求解器配置
│   │   │   └── ...                    # 其他配置文件
│   │   └── launch/                    # MoveIt启动文件
│   │
│   └── simple_robot_bringup/          # 系统启动包
│       └── launch/
│           └── demo.launch.py         # 主启动文件
│
└── README.md                          # 本文件
```

## 挑战任务

### 任务描述

编写一个C++的ROS 2节点,提供一个名为 `/move_to_position` 的服务。该服务:
- 接收一个`string`类型的位置名称(如"home", "pose1", "pose2")
- 根据预定义的关节角度,驱动机械臂运动到对应位置

### 技术要求

1. **语言**: C++
2. **框架**: ROS 2 Humble + MoveIt 2
3. **服务类型**: 自定义或使用std_srvs/srv/SetBool + Trigger组合
4. **硬编码位置**: 在程序中预先定义至少3个关节位置

### 机械臂信息

- **自由度**: 6轴
- **规划组名称**: `arm`
- **关节名称**: `joint1`, `joint2`, `joint3`, `joint4`, `joint5`, `joint6`
- **末端链接**: `link6`

### 示例关节角度(参考)

你可以在程序中定义类似以下的位置:

```cpp
// Home位置(全零位)
home: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

// 示例位置1
pose1: [0.5, -0.5, 1.0, 0.0, 1.5, 0.0]

// 示例位置2  
pose2: [-0.5, 0.5, -1.0, 0.0, -1.0, 0.5]
```

### 实现提示

1. **创建新的功能包**:
   ```bash
   cd ~/motion_challenge_ws/src
   ros2 pkg create --build-type ament_cmake arm_controller \
     --dependencies rclcpp moveit_ros_planning_interface
   ```

2. **核心API使用**:
   - 包含头文件: `#include <moveit/move_group_interface/move_group_interface.h>`
   - 创建MoveGroupInterface对象
   - 使用 `setJointValueTarget()` 设置目标关节角度
   - 使用 `plan()` 和 `execute()` 进行规划和执行

3. **服务定义**:
   可以创建自定义服务,或使用简单的字符串话题进行测试

### 测试你的节点

```bash
# 终端1: 启动demo环境
ros2 launch simple_robot_bringup demo.launch.py

# 终端2: 启动你的控制节点
ros2 run arm_controller position_controller_node

# 终端3: 测试服务调用
ros2 service call /move_to_position <your_service_type> "{position: 'home'}"
```

## 思考题

**问题**: 在你的实现中,机械臂是沿着直线运动到目标点的吗?如果不是,请解释为什么,并说明MoveIt中的"关节空间规划"和"笛卡尔空间规划"有何不同。

**提示**: 观察机械臂末端执行器在运动过程中的轨迹路径,思考关节插值和笛卡尔空间插值的区别。

## 调试技巧

### 查看话题和服务

```bash
# 查看所有话题
ros2 topic list

# 查看所有服务  
ros2 service list

# 查看move_group服务
ros2 service list | grep move_group
```

### 查看TF变换

```bash
# 查看坐标系关系
ros2 run tf2_tools view_frames
```

### RViz不显示机械臂?

检查以下配置:
- Fixed Frame设置为 `base_link` 或 `world`
- 添加RobotModel显示项
- 添加MotionPlanning显示项

## 参考资源

- [MoveIt 2 Tutorials](https://moveit.picknik.ai/humble/index.html)
- [MoveIt 2 C++ API文档](https://moveit.picknik.ai/humble/api/html/index.html)
- [ROS 2 Humble文档](https://docs.ros.org/en/humble/index.html)

## 常见问题

### Q: 编译失败,提示找不到MoveIt?
A: 确保已安装 `ros-humble-moveit` 和相关依赖包。

### Q: 启动后RViz黑屏或没有显示?
A: 检查是否正确source了工作区: `source install/setup.bash`

### Q: 如何获取当前关节角度?
A: 在RViz中查看,或使用命令: `ros2 topic echo /joint_states`

## 交付要求

1. **代码仓库**: 包含你编写的C++功能包
2. **README.md**: 说明如何编译和运行你的节点
3. **效果录屏**: 用于展示你的节点启动，机械臂控制等一系列操作
4. **回答思考题**: 关于关节空间vs笛卡尔空间规划

## 许可证

Apache-2.0

## 联系方式

发给组长邮箱quixoticmaker@163.com，或者QQ和组长私信
---

**祝你编码愉快! 🚀🦾**