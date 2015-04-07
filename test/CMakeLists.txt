
add_executable(create_iros_db create_iros_db.cpp)

target_link_libraries(create_iros_db ${catkin_LIBRARIES} sqlite3)

add_executable(make_table_grasps grasp_storage/table_grasp_main.cpp grasp_storage/table_grasp_maker.cpp)
target_link_libraries(make_table_grasps ${catkin_LIBRARIES} ${PROJECT_NAME})
add_dependencies(make_table_grasps dual_manipulation_shared_gencpp)