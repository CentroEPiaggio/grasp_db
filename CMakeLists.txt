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

add_subdirectory(db_apps)

#########
# TESTS #
#########

add_subdirectory(test)
