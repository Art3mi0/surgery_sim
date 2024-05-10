cmake_minimum_required(VERSION 3.0.2)
project(surgery_sim)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  std_msgs
  trajectory_msgs
  geometry_msgs
  tf
  tf2
  tf2_ros
  tf2_geometry_msgs
  message_generation  
  omni_msgs
  pcl_ros
  sensor_msgs
  cv_bridge
  image_transport
  image_geometry
)

add_message_files(
  FILES
	Plan.msg
	PedalEvent.msg
)

add_service_files(
  FILES
	Reset.srv
)

generate_messages(
  DEPENDENCIES
	geometry_msgs
	std_msgs
	sensor_msgs
)

catkin_package(
CATKIN_DEPENDS roscpp rospy roslib std_msgs geometry_msgs tf2 tf2_ros trajectory_msgs message_runtime sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

find_package(PCL 1.2 REQUIRED)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable(frame_tf_broadcaster src/frame_tf_broadcaster.cpp)
target_link_libraries(frame_tf_broadcaster ${catkin_LIBRARIES})

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})

add_executable(plan_listener src/plan_listener.cpp)
target_link_libraries(plan_listener ${catkin_LIBRARIES})

add_executable(point_counter src/point_counter.cpp)
target_link_libraries(point_counter ${catkin_LIBRARIES})

add_executable(data_logger src/data_logger.cpp)
target_link_libraries(data_logger ${catkin_LIBRARIES})

add_executable(switch_node src/switch_node.cpp)
target_link_libraries(switch_node ${catkin_LIBRARIES})

add_executable(switch_node_no_ff src/switch_node_no_ff.cpp)
target_link_libraries(switch_node_no_ff ${catkin_LIBRARIES})

add_executable(task_space_traj_reset src/task_space_traj_reset.cpp)
target_link_libraries(task_space_traj_reset ${catkin_LIBRARIES} ReflexxesTypeII)

add_executable(key_test src/key_test.cpp)
target_link_libraries(key_test ${catkin_LIBRARIES})

add_executable(pedal_controller src/pedal_controller.cpp)
target_link_libraries(pedal_controller ${catkin_LIBRARIES})

add_executable(user_overlay src/user_overlay.cpp)
target_link_libraries(user_overlay ${catkin_LIBRARIES})

add_executable(overlay_broadcast src/overlay_broadcast.cpp)
target_link_libraries(overlay_broadcast ${catkin_LIBRARIES})

##add_executable(reset_server src/reset_server.cpp)
##target_link_libraries(reset_server ${catkin_LIBRARIES})
##add_dependencies(reset_server surgery_sim_gencpp)
