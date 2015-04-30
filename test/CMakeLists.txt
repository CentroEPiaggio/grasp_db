add_executable(make_table_grasps table_grasp_main.cpp ../grasp_creation_utilities/table_grasp_maker.cpp)
target_link_libraries(make_table_grasps ${catkin_LIBRARIES} sqlite3 )

add_executable(make_specular_grasps specular_grasp_main.cpp ../grasp_creation_utilities/specular_grasp_maker.cpp)
target_link_libraries(make_specular_grasps ${catkin_LIBRARIES} sqlite3 )

add_executable(make_automatic_transitions automatic_transitions_main.cpp ../grasp_creation_utilities/named_automatic_transitions.cpp)
target_link_libraries(make_automatic_transitions ${catkin_LIBRARIES} sqlite3 )
