cmake_minimum_required(VERSION 3.8)
project(arm_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找编译我们代码所需的所有外部“零件包”
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(std_srvs REQUIRED)

# 添加我们要构建的“最终产品”（可执行文件）
add_executable(
  position_controller_node
  src/position_controller_node.cpp
)

# 告诉链接器，在“组装”时需要链接哪些外部零件
ament_target_dependencies(
  position_controller_node
  rclcpp
  moveit_ros_planning_interface
  std_srvs
)

# “安装”最终产品，以便 "ros2 run" 命令可以找到它
install(
  TARGETS position_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()