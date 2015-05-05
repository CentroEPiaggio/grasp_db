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

#################
# CATKIN CONFIG #
#################

find_package(catkin REQUIRED COMPONENTS rviz roscpp roslib dual_manipulation_shared kdl_conversions rosbag)
catkin_package()
include_directories(include ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

#########
# BUILD #
#########

# NOT USED
#include_directories(${catkin_INCLUDE_DIRS})
#set(SOURCE_FILES src/ros_server.cpp src/database_manager.cpp
#  src/main.cpp
#)
#add_executable(dual_manipulation_grasp_DB ${SOURCE_FILES})
#target_link_libraries(dual_manipulation_grasp_DB ${catkin_LIBRARIES} sqlite3)

### CREATE/COMPLETE DATABASES

# CREATION UTILITIES
include_directories(grasp_creation_utilities/include)
add_library(grasp_creation_utilities
	grasp_creation_utilities/table_grasp_maker.cpp
	grasp_creation_utilities/specular_grasp_maker.cpp
	grasp_creation_utilities/named_automatic_transitions.cpp
)
target_link_libraries(grasp_creation_utilities
	${catkin_LIBRARIES}
	sqlite3
)

# DEPRECATED
# add_subdirectory(db_apps)
# add_executable(create_iros_db create_iros_db.cpp)
# target_link_libraries(create_iros_db ${catkin_LIBRARIES} sqlite3 grasp_creation_utilities)
# add_dependencies(create_iros_db grasp_creation_utilities)

# SINGLE DBs
add_executable(create_containerB_db
	db_apps/create_containerB_db.cpp)
add_dependencies(create_containerB_db
	grasp_creation_utilities
)
target_link_libraries(create_containerB_db 
	${catkin_LIBRARIES}
	sqlite3 
	grasp_creation_utilities
)

# THIS GROUPS AVAILABLE DATABASES INTO A SINGLE ONE
add_executable(create_full_db 
	db_apps/create_full_db.cpp
)
add_dependencies(create_full_db
	dual_manipulation_shared
)
target_link_libraries(create_full_db
	${catkin_LIBRARIES}
	sqlite3
)

# GRASPS SERIALIZER
add_library(grasps_serializer
	grasps_serializer/grasps_serializer.cpp
)
add_dependencies(grasps_serializer
	dual_manipulation_shared
)
target_link_libraries(grasps_serializer
	dual_manipulation_shared
	${catkin_LIBRARIES}
)

add_executable(grasps_serializer_node
	grasps_serializer/main.cpp
)
target_link_libraries(grasps_serializer_node
	grasps_serializer
	${catkin_LIBRARIES}
)

# MODIFICATION UTILIY FOR REFINEMENT
add_executable(grasp_modification_utility
	grasp_modification_utility/grasp_modification_utility.cpp
	grasp_modification_utility/main.cpp)
add_dependencies(grasp_modification_utility
	dual_manipulation_shared
)
target_link_libraries(grasp_modification_utility
	${catkin_LIBRARIES}
	sqlite3
)


#########
# TESTS #
#########

add_subdirectory(test)
