cmake_minimum_required(VERSION 3.8)
project(roslinker)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tb_interfaces REQUIRED)

set(ATRBRIDGE_CORE_DIR "/home/shaon/mini-turtlebot/atrbridge_core" CACHE PATH "Path to atrbridge_core project")
if(NOT EXISTS "${ATRBRIDGE_CORE_DIR}/CMakeLists.txt")
  message(FATAL_ERROR "atrbridge_core not found at ${ATRBRIDGE_CORE_DIR}")
endif()

set(ATRBRIDGE_BUILD_DAEMON OFF CACHE BOOL "Build core daemon from nested build" FORCE)
set(ATRBRIDGE_BUILD_WS_GATEWAY OFF CACHE BOOL "Build core ws gateway from nested build" FORCE)
add_subdirectory("${ATRBRIDGE_CORE_DIR}" "${CMAKE_CURRENT_BINARY_DIR}/atrbridge_core_build")

add_executable(roslinker_node src/roslinker_node.cpp)
target_include_directories(roslinker_node PRIVATE "${ATRBRIDGE_CORE_DIR}/include")
target_link_libraries(roslinker_node atrbridge_core)
ament_target_dependencies(
  roslinker_node
  rclcpp
  geometry_msgs
  sensor_msgs
  tb_interfaces
)

install(TARGETS roslinker_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
