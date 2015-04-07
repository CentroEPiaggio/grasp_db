
add_executable(create_iros_db create_iros_db.cpp)

target_link_libraries(create_iros_db ${catkin_LIBRARIES} sqlite3)