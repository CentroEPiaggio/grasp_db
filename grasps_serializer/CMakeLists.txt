# GRASPS SERIALIZER APP
add_library(grasps_serializer
	grasps_serializer.cpp
)
add_dependencies(grasps_serializer
	dual_manipulation_shared
)
target_link_libraries(grasps_serializer
	dual_manipulation_shared
	${catkin_LIBRARIES}
)

add_executable(grasps_serializer_node
	main.cpp
)
target_link_libraries(grasps_serializer_node
	grasps_serializer
	${catkin_LIBRARIES}
)
