cmake_minimum_required(VERSION 2.8.3)
project(dual_manipulation_grasp_DB)

find_package(catkin REQUIRED COMPONENTS rviz roscpp dual_manipulation_shared)
catkin_package()
include_directories(include ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

message(${catkin_INCLUDE_DIRS})

include_directories(${catkin_INCLUDE_DIRS})
set(SOURCE_FILES src/ros_server.cpp src/database_manager.cpp
  src/main.cpp
)

add_executable(dual_manipulation_grasp_DB ${SOURCE_FILES})

target_link_libraries(dual_manipulation_grasp_DB ${catkin_LIBRARIES} sqlite3)
