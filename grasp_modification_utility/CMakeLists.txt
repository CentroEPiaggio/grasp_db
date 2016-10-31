# GRASP MODIFICATION UTILITY (GMU)

find_package(catkin REQUIRED COMPONENTS 
    rviz
    roscpp
    dual_manipulation_shared
    kdl_conversions
)
link_directories(${catkin_LIBRARY_DIRS})

if($ENV{ROS_DISTRO} STRLESS "indigo" OR $ENV{ROS_DISTRO} STREQUAL "indigo")
    find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
    include(${QT_USE_FILE})
    # wrap using Qt
    qt4_wrap_cpp(GMU_GUI_MOC_FILES
        GMU_gui.h
    )
elseif($ENV{ROS_DISTRO} STRGREATER "indigo")
    find_package(Qt5 COMPONENTS Core Widgets REQUIRED)
    include_directories(${Qt5Core_INCLUDE_DIRS} ${Qt5Widgets_INCLUDE_DIRS})
    add_definitions(-DQT_NO_KEYWORDS)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${Qt5Widgets_EXECUTABLE_COMPILE_FLAGS}")
    # wrap using Qt
    qt5_wrap_cpp(GMU_GUI_MOC_FILES
        GMU_gui.h
    )
    SET(QT_LIBRARIES
        ${Qt5Core_LIBRARIES}
        ${Qt5Widgets_LIBRARIES}
    )
endif()

set(GMU_GUI_SRC
    ${GMU_GUI_MOC_FILES}
    GMU_gui.cpp
)

add_executable(grasp_modification_utility ${GMU_GUI_SRC}
    grasp_modification_utility.cpp
    main.cpp
)
add_dependencies(grasp_modification_utility
    dual_manipulation_shared
)
target_link_libraries(grasp_modification_utility
    ${catkin_LIBRARIES} ${QT_LIBRARIES}
)

add_library(rviz_gmu_panel rviz_gmu_panel.cpp grasp_modification_utility.cpp ${GMU_GUI_SRC})

target_link_libraries(rviz_gmu_panel ${QT_LIBRARIES} ${catkin_LIBRARIES})

install(FILES plugin_description.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
