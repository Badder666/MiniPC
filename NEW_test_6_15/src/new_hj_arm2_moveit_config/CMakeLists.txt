cmake_minimum_required(VERSION 3.22)
project(new_hj_arm2_moveit_config)

find_package(ament_cmake REQUIRED)

ament_package()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

# Gazebo 仿真相关
install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME})
install(DIRECTORY scripts DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS)
