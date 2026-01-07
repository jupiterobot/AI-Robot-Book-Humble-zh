cmake_minimum_required(VERSION 3.5)
project(happy_parameter)

# 依存関係を見つける
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

set(dependencies
  rclcpp
  rcl_interfaces
  std_msgs
)


# インストールするPythonスクリプトを指定
install(PROGRAMS
  scripts/happy_parameter_client_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# amentパッケージの設定
ament_package()
