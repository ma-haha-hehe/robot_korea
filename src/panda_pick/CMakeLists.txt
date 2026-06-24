cmake_minimum_required(VERSION 3.8)
project(panda_pick)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. 寻找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(franka_msgs REQUIRED) # 如果你不用 franka 建议删掉，防止干扰
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometric_shapes REQUIRED) # 记得加上这个
find_package(ur_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# 2. 定义可执行文件 (Target 名称定为 cpp_pick_node_ur5)
add_executable(cpp_pick_node_ur5 src/cpp_pick_node_ur5.cpp)

# 3. 链接依赖 (第一个参数必须和上面 add_executable 的名字完全一致)
ament_target_dependencies(cpp_pick_node_ur5
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  moveit_msgs
  tf2
  tf2_geometry_msgs
  franka_msgs
  rclcpp_action
  control_msgs
  ur_msgs
  std_msgs
)

# 链接普通 C++ 库
target_link_libraries(cpp_pick_node_ur5
  yaml-cpp 
  geometric_shapes::geometric_shapes
)

# 4. 安装规则 (注意 TARGETS 后面的名字也要对应)
install(TARGETS
  cpp_pick_node_ur5
  DESTINATION lib/${PROJECT_NAME}
)

# 安装其他资源
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
