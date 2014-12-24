cmake_minimum_required(VERSION 2.8.3)
project(dual_manipulation_grasp_DB)

ENABLE_LANGUAGE(CXX)

include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()


find_package(catkin REQUIRED COMPONENTS roscpp roslib dual_manipulation_shared)
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
