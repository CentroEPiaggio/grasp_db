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

if($ENV{ROS_DISTRO} STRLESS "indigo" OR $ENV{ROS_DISTRO} STREQUAL "indigo")
    find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
    include(${QT_USE_FILE})
elseif($ENV{ROS_DISTRO} STRGREATER "indigo")
    find_package(Qt5 COMPONENTS Core Widgets REQUIRED)
    include_directories(${Qt5Core_INCLUDE_DIRS} ${Qt5Widgets_INCLUDE_DIRS})
    add_definitions(-DQT_NO_KEYWORDS)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${Qt5Widgets_EXECUTABLE_COMPILE_FLAGS}")
endif()

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
    grasp_creation_utilities/symmetric_grasp_maker.cpp
        grasp_creation_utilities/named_automatic_transitions.cpp #DEPRECATED
	grasp_creation_utilities/geometric_automatic_transitions.cpp
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

# BOOTSTRAP APPs
# ToDO: create a loop with source names
add_executable(bootstrap_containerB_db
	db_apps/bootstrap_containerB_db.cpp)
add_dependencies(bootstrap_containerB_db
	grasp_creation_utilities
)
target_link_libraries(bootstrap_containerB_db
	${catkin_LIBRARIES}
	sqlite3
	grasp_creation_utilities
)

add_executable(bootstrap_mugD_db
	db_apps/bootstrap_mugD_db.cpp)
add_dependencies(bootstrap_mugD_db
	grasp_creation_utilities
)
target_link_libraries(bootstrap_mugD_db
	${catkin_LIBRARIES}
	sqlite3
	grasp_creation_utilities
)

add_executable(bootstrap_bowlA_db
	db_apps/bootstrap_bowlA_db.cpp)
add_dependencies(bootstrap_bowlA_db
	grasp_creation_utilities
)
target_link_libraries(bootstrap_bowlA_db
	${catkin_LIBRARIES}
	sqlite3
	grasp_creation_utilities
)

add_executable(bootstrap_bowlB_db
	db_apps/bootstrap_bowlB_db.cpp)
add_dependencies(bootstrap_bowlB_db
	grasp_creation_utilities
)
target_link_libraries(bootstrap_bowlB_db
	${catkin_LIBRARIES}
	sqlite3
	grasp_creation_utilities
)

add_executable(bootstrap_containerA_db
    db_apps/bootstrap_containerA_db.cpp)
add_dependencies(bootstrap_containerA_db
    grasp_creation_utilities
)
target_link_libraries(bootstrap_containerA_db
    ${catkin_LIBRARIES}
    sqlite3
    grasp_creation_utilities
)

add_executable(bootstrap_cylinder_db
    db_apps/bootstrap_cylinder_db.cpp)
add_dependencies(bootstrap_cylinder_db
    grasp_creation_utilities
)
target_link_libraries(bootstrap_cylinder_db
    ${catkin_LIBRARIES}
    sqlite3
    grasp_creation_utilities
)

add_executable(bootstrap_ball_db
    db_apps/bootstrap_ball_db.cpp)
add_dependencies(bootstrap_ball_db
    grasp_creation_utilities
)
target_link_libraries(bootstrap_ball_db
    ${catkin_LIBRARIES}
    sqlite3
    grasp_creation_utilities
)

# CREATE FULL DATABASE APP
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

# APPLY BASIC GEOMETRIC FILTER FOR ALL TO ALL TRANSITIONS
add_executable(apply_geometric_transitions
	db_apps/applyGeometricTransitions.cpp)
add_dependencies(apply_geometric_transitions
	grasp_creation_utilities
)
target_link_libraries(apply_geometric_transitions
	${catkin_LIBRARIES}
	sqlite3
	grasp_creation_utilities
)

# GRASP MODIFICATION UTILITY (GMU)
if($ENV{ROS_DISTRO} STRLESS "indigo" OR $ENV{ROS_DISTRO} STREQUAL "indigo")
    qt4_wrap_cpp(GMU_GUI_MOC_FILES
    grasp_modification_utility/GMU_gui.h)
elseif($ENV{ROS_DISTRO} STRGREATER "indigo")
    qt5_wrap_cpp(GMU_GUI_MOC_FILES
    grasp_modification_utility/GMU_gui.h)
    SET(QT_LIBRARIES
        ${Qt5Core_LIBRARIES}
        ${Qt5Widgets_LIBRARIES}
    )
endif()

set(GMU_GUI_SRC
${GMU_GUI_MOC_FILES}
grasp_modification_utility/GMU_gui.cpp)

add_executable(grasp_modification_utility ${GMU_GUI_SRC}
	grasp_modification_utility/grasp_modification_utility.cpp
	grasp_modification_utility/main.cpp)
add_dependencies(grasp_modification_utility
	dual_manipulation_shared
)
target_link_libraries(grasp_modification_utility
	${catkin_LIBRARIES} ${QT_LIBRARIES}
	sqlite3
)

add_library(rviz_gmu_panel grasp_modification_utility/rviz_gmu_panel.cpp grasp_modification_utility/grasp_modification_utility.cpp ${GMU_GUI_SRC})

target_link_libraries(rviz_gmu_panel ${QT_LIBRARIES} ${catkin_LIBRARIES})

install(FILES plugin_description.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

add_executable(create_3vito_db
        db_apps/create_3vito_db.cpp db_apps/multi_arm_cylinder_db.cpp
)
add_dependencies(create_3vito_db
        dual_manipulation_shared
        apply_geometric_transitions
        grasp_creation_utilities
)

target_link_libraries(create_3vito_db
        ${catkin_LIBRARIES}
        sqlite3
        grasp_creation_utilities
)

add_executable(apply_named_transitions
        db_apps/apply_named_transitions.cpp
)
add_dependencies(apply_named_transitions
        dual_manipulation_shared
        grasp_creation_utilities
)

target_link_libraries(apply_named_transitions
        ${catkin_LIBRARIES}
        sqlite3
        grasp_creation_utilities
        dual_manipulation_shared
)


add_executable(create_simple_factory_db
        db_apps/create_simple_factory_db.cpp
)
add_dependencies(create_simple_factory_db
        dual_manipulation_shared
        apply_geometric_transitions
        grasp_creation_utilities
)

target_link_libraries(create_simple_factory_db
        ${catkin_LIBRARIES}
        sqlite3
        grasp_creation_utilities
)


add_executable(create_bi_tino_simple_factory_db
        db_apps/create_bi_tino_simple_factory_db.cpp
)
add_dependencies(create_bi_tino_simple_factory_db
        dual_manipulation_shared
        apply_geometric_transitions
        grasp_creation_utilities
)
target_link_libraries(create_bi_tino_simple_factory_db
        ${catkin_LIBRARIES}
        sqlite3
        grasp_creation_utilities
)

add_executable(create_house_db
        db_apps/create_house_db.cpp
)
add_dependencies(create_house_db
        dual_manipulation_shared
        apply_geometric_transitions
        grasp_creation_utilities
)
target_link_libraries(create_house_db
        ${catkin_LIBRARIES}
        sqlite3
        grasp_creation_utilities
)

add_subdirectory(db_apps)

#########
# TESTS #
#########

add_subdirectory(test)
