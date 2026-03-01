cmake_minimum_required(VERSION 3.16)
project(atrbridge_core VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

option(ATRBRIDGE_BUILD_DAEMON "Build atrbridge core daemon binary" ON)
option(ATRBRIDGE_BUILD_WS_GATEWAY "Build atrbridge ws/json gateway binary" ON)

add_library(atrbridge_core
  src/framing.cpp
  src/transport_tcp.cpp
  src/core.cpp
)

target_include_directories(atrbridge_core
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_options(atrbridge_core PRIVATE -Wall -Wextra -Wpedantic)

if(UNIX AND NOT APPLE)
  target_link_libraries(atrbridge_core PRIVATE pthread)
endif()

if(ATRBRIDGE_BUILD_DAEMON)
  add_executable(atrbridge_core_daemon src/daemon_main.cpp)
  target_link_libraries(atrbridge_core_daemon PRIVATE atrbridge_core)
endif()

if(ATRBRIDGE_BUILD_WS_GATEWAY)
  find_package(Boost REQUIRED COMPONENTS system)
  add_executable(atrbridge_ws_gateway src/ws_gateway_main.cpp)
  target_link_libraries(atrbridge_ws_gateway PRIVATE atrbridge_core Boost::system)
endif()
