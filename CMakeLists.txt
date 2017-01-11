cmake_minimum_required(VERSION 2.8.3)
project(dual_manipulation_grasp_db)

ENABLE_LANGUAGE(CXX)
SET(CMAKE_BUILD_TYPE "Debug")

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

#################
# CATKIN CONFIG #
#################

find_package(catkin REQUIRED COMPONENTS 
    roscpp
    roslib
    dual_manipulation_shared
    kdl_conversions
)
catkin_package()
include_directories(include ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

#########
# BUILD #
#########

### CREATE/COMPLETE DATABASES

# CREATION UTILITIES
add_library(grasp_creation_utilities
    src/table_grasp_maker.cpp
    src/specular_grasp_maker.cpp
    src/symmetric_grasp_maker.cpp
    src/named_automatic_transitions.cpp
    src/geometric_automatic_transitions.cpp
    src/create_db_utils.cpp
)
target_link_libraries(grasp_creation_utilities
    ${catkin_LIBRARIES}
)

add_subdirectory(grasp_modification_utility)

add_subdirectory(db_apps)

#########
# TESTS #
#########

add_subdirectory(test)
