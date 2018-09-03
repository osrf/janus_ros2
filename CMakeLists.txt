cmake_minimum_required(VERSION 3.5)
project(janus_ros2)

if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)  # -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
#find_package(PkgConfig REQUIRED)

#pkg_search_module(glib REQUIRED glib-2.0)
#pkg_search_module(nice REQUIRED nice)
#pkg_search_module(libsrtp2 REQUIRED libsrtp2)
#pkg_search_module(jansson REQUIRED jansson)
#pkg_search_module(libssl REQUIRED libssl)
#pkg_search_module(libcrypto REQUIRED libcrypto)
#pkg_search_module(libwebsockets REQUIRED libwebsockets)

find_path(JANUS_INCLUDE_PATH NAMES "janus/transports/transport.h")
if (NOT JANUS_INCLUDE_PATH)
  message(FATAL_ERROR "Unable to find Janus; can't build plugin")
else()
  message("Found Janus include path: ${JANUS_INCLUDE_PATH}")
endif()

#target_include_directories(janus PRIVATE ${glib_INCLUDE_DIRS} ${nice_INCLUDE_DIRS} ${libsrtp2_INCLUDE_DIRS})
#target_include_directories(janus PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

add_library(janus_ros2_transport_plugin MODULE transports/janus_ros2.cpp)
target_include_directories(janus_ros2_transport_plugin ${JANUS_INCLUDE_PATH})
target_include_directories(janus_ros2_transport_plugin PRIVATE ${rclcpp_INCLUDE_DIRS})
ament_target_dependencies(janus_ros2_transport_plugin rclcpp std_msgs)

#target_include_directories(janus_ros2 PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
#target_include_directories(janus_ros2 PRIVATE ${glib_INCLUDE_DIRS} ${nice_INCLUDE_DIRS} ${libsrtp2_INCLUDE_DIRS})

ament_package()
