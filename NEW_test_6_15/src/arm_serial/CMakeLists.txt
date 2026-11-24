cmake_minimum_required(VERSION 3.5)
project(arm_serial)

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# 生成自定义消息
rosidl_generate_interfaces(${PROJECT_NAME}
  msg/CCtrlPayload.msg
  msg/CCtrlJoint.msg
  msg/TcInfo.msg
  DEPENDENCIES std_msgs
)

# 安装 Python 脚本
install(DIRECTORY
    scripts/
  DESTINATION lib/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

# 也安装到 share 目录以便 ros2 run 查找
install(DIRECTORY
    scripts/
  DESTINATION share/${PROJECT_NAME}/scripts
)

# 安装 launch 文件
install(DIRECTORY
    launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
