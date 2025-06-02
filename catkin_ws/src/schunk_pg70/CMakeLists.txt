cmake_minimum_required(VERSION 2.8.3)
project(schunk_pg70)

find_package(catkin REQUIRED COMPONENTS
	actionlib
	actionlib_msgs
	roscpp
	serial
	std_msgs
	sensor_msgs
	message_generation
	tf
)

find_package(Boost REQUIRED COMPONENTS system)


include_directories(${catkin_INCLUDE_DIRS}
                    ${PROJECT_SOURCE_DIR}/include/)
                   
SET(SOURCES ${PROJECT_SOURCE_DIR}/src/pg70_rs232_control_node.cpp
    ${PROJECT_SOURCE_DIR}/src/pg70_rs232_control_lib.cpp
    ${PROJECT_SOURCE_DIR}/src/pg70_grasp_action_server.cpp
    ${PROJECT_SOURCE_DIR}/src/pg70_move_action_server.cpp
)
   
SET(HEADERS 
    ${PROJECT_SOURCE_DIR}/include/pg70_rs232_control.h
	  ${PROJECT_SOURCE_DIR}/include/pg70_action_server/pg70_grasp_action_server.h
    ${PROJECT_SOURCE_DIR}/include/pg70_action_server/pg70_move_action_server.h
)

add_message_files(FILES 
  message_control_position.msg
)

add_service_files(FILES 
      reference.srv
      set_position.srv
			set_pvac.srv
      get_error.srv
      get_position.srv
      get_current.srv
      acknowledge_error.srv
			move_time.srv
			move_time_loop.srv
      stop.srv
)

add_action_files( FILES 
	Grasp.action
	Move.action
)

generate_messages(DEPENDENCIES 
  std_msgs
  sensor_msgs
  geometry_msgs
  actionlib_msgs
  )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
  DEPENDS roscpp serial tf
  CATKIN_DEPENDS message_runtime actionlib_msgs
)


include_directories(include 
	${catkin_INCLUDE_DIRS} 
	${Boost_INCLUDE_DIRS}
)


add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} schunk_pg70_gencpp )
target_link_libraries(${PROJECT_NAME} ${ROS_LIBRARIES} ${catkin_LIBRARIES} )


add_executable(pg70_control_position src/pg70_control_position.cpp)
target_link_libraries(pg70_control_position ${catkin_LIBRARIES})
add_dependencies(pg70_control_position schunk_pg70_gencpp)

