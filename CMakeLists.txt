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

# at least for Ubuntu 16.04, we need a newer version of libsrtp
# so we'll build it here. In the future, we could be smarter about
# this and check the version provided by the system.
macro(build_srtp)
  include(ExternalProject)
  ExternalProject_Add(SRTP-2.2.0
    URL https://github.com/cisco/libsrtp/archive/v2.2.0.tar.gz
    URL_MD5 f77a27457d219f2991ea7aa2f0c11ec9
    LOG_DOWNLOAD ON
    LOG_CONFIGURE ON
    LOG_BUILD ON
    CONFIGURE_COMMAND
      <SOURCE_DIR>/configure
      --prefix=${CMAKE_CURRENT_BINARY_DIR}/libsrtp
      --enable-openssl
    BUILD_COMMAND make shared_library
    INSTALL_COMMAND make install
  )
  install(
    DIRECTORY   ${CMAKE_CURRENT_BINARY_DIR}/libsrtp/include
    DESTINATION ${CMAKE_INSTALL_PREFIX}
  )
  install(
    DIRECTORY   ${CMAKE_CURRENT_BINARY_DIR}/libsrtp/lib
    DESTINATION ${CMAKE_INSTALL_PREFIX}
  )
endmacro()

macro(build_janus)
  include(ExternalProject)
  ExternalProject_Add(JANUS-0.4.2
    URL https://github.com/codebot/janus-gateway/archive/v0.4.2.zip
    URL_MD5 ffa3b2276bf61f471844d0bfe8666340
    #LOG_DOWNLOAD ON
    #LOG_CONFIGURE ON
    #LOG_BUILD ON
    BUILD_IN_SOURCE 1
    CONFIGURE_COMMAND <SOURCE_DIR>/autogen.sh
    COMMAND <SOURCE_DIR>/configure
      CPPFLAGS=-I${CMAKE_CURRENT_BINARY_DIR}/libsrtp/include
      LDFLAGS=-L${CMAKE_CURRENT_BINARY_DIR}/libsrtp/lib
      --prefix=${CMAKE_CURRENT_BINARY_DIR}/janus
      --disable-plugin-audiobridge
      --disable-plugin-echotest
      --disable-plugin-lua
      --disable-plugin-nosip
      --disable-plugin-recordplay
      --disable-plugin-sip
      --disable-plugin-sipre
      --disable-plugin-streaming
      --disable-plugin-textroom
      --disable-plugin-videocall
      --disable-plugin-videoroom
      --disable-plugin-voicemail
      --disable-rabbitmq
      --disable-data-channels
      --disable-mqtt
      --disable-rabbitmq-event-handler
      --disable-rest
      --disable-unix-sockets
      --disable-websockets
      --enable-post-processing
    BUILD_COMMAND make
    INSTALL_COMMAND make install
  )
  add_dependencies(JANUS-0.4.2 SRTP-2.2.0)
  install(
    DIRECTORY   ${CMAKE_CURRENT_BINARY_DIR}/janus/
    DESTINATION ${CMAKE_INSTALL_PREFIX}
    USE_SOURCE_PERMISSIONS
  )
endmacro()

build_srtp()
build_janus()

find_package(PkgConfig REQUIRED)

pkg_search_module(glib REQUIRED glib-2.0)

ExternalProject_Get_Property(JANUS-0.4.2 source_dir)
set(JANUS_SOURCE_DIR ${source_dir})

add_library(janus_ros2 MODULE janus_ros2.cpp)
target_include_directories(janus_ros2 PRIVATE ${CMAKE_INSTALL_PREFIX}/include ${JANUS_SOURCE_DIR} ${glib_INCLUDE_DIRS} ${rclcpp_INCLUDE_DIRS})
ament_target_dependencies(janus_ros2 rclcpp std_msgs)
add_dependencies(janus_ros2 JANUS-0.4.2)

install(
  TARGETS janus_ros2
  DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)

ament_package()
