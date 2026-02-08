cmake_minimum_required(VERSION 3.8)
project(tb_bridge_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(tb_interfaces REQUIRED)

include_directories(include)

add_executable(tb_bridge_node
  src/main.cpp
  src/bridge_node.cpp
  src/framing.cpp
  src/transport_tcp.cpp
)

ament_target_dependencies(tb_bridge_node
  rclcpp
  geometry_msgs
  sensor_msgs
  nav_msgs
  std_msgs
  builtin_interfaces
  tb_interfaces
)

install(TARGETS tb_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_package()

