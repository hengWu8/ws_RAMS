cmake_minimum_required(VERSION 3.10)
project(click_to_move)

# C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

# 可执行程序
add_executable(click_to_move_node src/click_to_move_node.cpp)
ament_target_dependencies(click_to_move_node
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  moveit_ros_planning_interface
)

# 安装可执行文件
install(TARGETS click_to_move_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# 安装 launch 文件（含 system_bringup.launch.py）
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
