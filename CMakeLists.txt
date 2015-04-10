cmake_minimum_required(VERSION 2.8.3)
project(dual_manipulation_grasp_db)

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


find_package(catkin REQUIRED COMPONENTS roscpp roslib dual_manipulation_shared kdl_conversions)
catkin_package()
include_directories(include ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

message(${catkin_INCLUDE_DIRS})

include_directories(${catkin_INCLUDE_DIRS})
set(SOURCE_FILES src/ros_server.cpp src/database_manager.cpp
  src/main.cpp
)

add_executable(grasp_storage grasp_storage/grasp_storage.cpp grasp_storage/main.cpp)
target_link_libraries(grasp_storage ${catkin_LIBRARIES} dual_manipulation_shared)
add_dependencies(grasp_storage dual_manipulation_shared)

add_executable(dual_manipulation_grasp_DB ${SOURCE_FILES})
target_link_libraries(dual_manipulation_grasp_DB ${catkin_LIBRARIES} sqlite3)


add_executable(create_iros_db test/create_iros_db.cpp test/table_grasp_maker.cpp)

target_link_libraries(create_iros_db ${catkin_LIBRARIES} sqlite3)

add_executable(make_table_grasps test/table_grasp_main.cpp test/table_grasp_maker.cpp)
target_link_libraries(make_table_grasps ${catkin_LIBRARIES} sqlite3 )

add_executable(make_specular_grasps test/specular_grasp_main.cpp test/specular_grasp_maker.cpp)
target_link_libraries(make_specular_grasps ${catkin_LIBRARIES} sqlite3 )

add_executable(make_automatic_transitions test/automatic_transitions_main.cpp test/named_automatic_transitions.cpp)
target_link_libraries(make_automatic_transitions ${catkin_LIBRARIES} sqlite3 )

add_executable(create_container_b_db test/create_containerB_db.cpp test/table_grasp_maker.cpp test/specular_grasp_maker.cpp test/named_automatic_transitions.cpp)
target_link_libraries(create_container_b_db ${catkin_LIBRARIES} sqlite3)
