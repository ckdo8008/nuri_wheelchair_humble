cmake_minimum_required(VERSION 3.8)
project(nuri_humble_bringup)

# C++ 14 사용
add_compile_options(-std=c++14)

# 패키지 의존성
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)
find_package(control_msgs REQUIRED)
find_package(controller_manager REQUIRED)
find_package(diff_drive_controller REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(joint_state_broadcaster REQUIRED)

# urdf 파일을 설치
install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}/
)

# config 파일을 설치
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

# launch 파일을 설치
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# ament 패키지 설정
ament_package()