cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)
project(windPlugin)

# --------------------------------------------------------------------------- #
# If ament_cmake is found build as an ament package, otherwise ignore.
# This is so the system may be built for Gazebo only, if ROS is not available.
# find_package(ament_cmake QUIET)
# if(${ament_cmake_FOUND})
#   message("Building ${PROJECT_NAME} as an `ament_cmake` project.")
# endif()

# --------------------------------------------------------------------------- #
# Compile as C++14.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -fPIC")



#这将强制将${CMAKE_INSTALL_PREFIX}设置为$HOME/.gz/sim/plugins，并且在安装时将文件安装到该目录下。
set(CMAKE_INSTALL_PREFIX "$ENV{HOME}/.gz/sim/plugins" CACHE PATH "Installation prefix" FORCE)

# # --------------------------------------------------------------------------- #
find_package(gz-cmake3 REQUIRED)

set(GZ_CMAKE_VER ${gz-cmake3_VERSION_MAJOR})

gz_find_package(gz-sim7 REQUIRED)
set(GZ_SIM_VER ${gz-sim7_VERSION_MAJOR})

find_package(catkin REQUIRED COMPONENTS  
  roscpp 
  message_generation   
  std_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
)

find_package(Eigen3 REQUIRED)

include_directories(
    ${catkin_INCLUDE_DIRS}
    Eigen3
    include
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime  roscpp 
)


# main plugin
add_library(${PROJECT_NAME}
  SHARED
  src/windPlugin.cc
)



target_link_libraries(${PROJECT_NAME}
  PRIVATE
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  # ${EIGEN}
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
  include
)


install(
  TARGETS
  windPlugin
  DESTINATION ~/.gz/sim/plugins
)



add_executable(run_ros_main src/run_ros_main.cpp)

 target_link_libraries(run_ros_main
   PUBLIC
   ${catkin_LIBRARIES}
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
 )
