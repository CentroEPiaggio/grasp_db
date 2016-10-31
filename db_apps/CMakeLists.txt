# db_apps CMake - create databases from scratch using few grasps as a starting point

# APPLY BASIC GEOMETRIC FILTER FOR ALL TO ALL TRANSITIONS
add_executable(apply_geometric_transitions
	applyGeometricTransitions.cpp)
add_dependencies(apply_geometric_transitions
	grasp_creation_utilities
)
target_link_libraries(apply_geometric_transitions
	${catkin_LIBRARIES}
	sqlite3
	grasp_creation_utilities
)

# CREATE TRANSITIONS BASED ON GRASP NAMES
add_executable(apply_named_transitions
        apply_named_transitions.cpp
)
add_dependencies(apply_named_transitions
        dual_manipulation_shared
        grasp_creation_utilities
)

# CREATE NEW DATABASES
add_executable(create_3vito_db
        create_3vito_db.cpp multi_arm_cylinder_db.cpp
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

target_link_libraries(apply_named_transitions
        ${catkin_LIBRARIES}
        sqlite3
        grasp_creation_utilities
        dual_manipulation_shared
)


add_executable(create_simple_factory_db
        create_simple_factory_db.cpp
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
        create_bi_tino_simple_factory_db.cpp
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
        create_house_db.cpp
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

# 
add_executable(create_box_soma_db
    create_box_soma_db.cpp
)
add_dependencies(create_box_soma_db
    dual_manipulation_shared
    grasp_creation_utilities
)
target_link_libraries(create_box_soma_db
    ${catkin_LIBRARIES}
    grasp_creation_utilities
)
