cmake_minimum_required(VERSION 3.0.2)
project(acri_localisation)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_transport
  nav_msgs
  message_generation
  mrpt_bridge
  pcl_conversions
  pcl_msgs
  pcl_ros
  pose_cov_ops
  rosbag
  roscpp
  rospy
  sensor_msgs 
  std_msgs 
  tf
)  
 
# Generate messages in the 'msg' folder
add_message_files( 
  FILES
  railLine.msg
  railLineVector.msg
  railPair.msg
  railPairVector.msg 
  railClosestPair.msg
  controlToNUC.msg 
  controlFromNUC.msg
) 

generate_messages(
  DEPENDENCIES
  geometry_msgs  
  std_msgs
)

find_package(MRPT REQUIRED)
message(STATUS "Found MRPT: " ${MRPT_VERSION})
if("${MRPT_VERSION}" VERSION_LESS "1.9.9")
	# MRPT<2.0
	set (CMAKE_CXX_STANDARD 11)
	find_package(MRPT REQUIRED base)
else()
	# MRPT>=2.0
	find_package(MRPT REQUIRED poses)
endif()
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
  cv_bridge
  geometry_msgs
  image_transport
  message_runtime
  mrpt_bridge
  nav_msgs
  pcl_conversions
  pcl_msgs
  pcl_ros
  pose_cov_ops
  rosbag
  roscpp
  rospy
  sensor_msgs
  std_msgs 
  tf
)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(${PROJECT_NAME}_odom_conversion src/odom_conversion.cpp)
set_target_properties(${PROJECT_NAME}_odom_conversion PROPERTIES OUTPUT_NAME odom_conversion PREFIX "")
target_link_libraries(${PROJECT_NAME}_odom_conversion ${catkin_LIBRARIES} ${MRPT_LIBRARIES})

add_executable(${PROJECT_NAME}_odom_publish src/odom_publish.cpp)
set_target_properties(${PROJECT_NAME}_odom_publish PROPERTIES OUTPUT_NAME odom_publish PREFIX "")
target_link_libraries(${PROJECT_NAME}_odom_publish ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_line_detection src/line_detection.cpp)
set_target_properties(${PROJECT_NAME}_line_detection PROPERTIES OUTPUT_NAME line_detection PREFIX "")
target_link_libraries(${PROJECT_NAME}_line_detection ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(${PROJECT_NAME}_rail_class src/rail_classification.cpp)
set_target_properties(${PROJECT_NAME}_rail_class PROPERTIES OUTPUT_NAME rail_class PREFIX "")
target_link_libraries(${PROJECT_NAME}_rail_class ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_rail_vis src/rail_visualisation.cpp)
set_target_properties(${PROJECT_NAME}_rail_vis PROPERTIES OUTPUT_NAME rail_vis PREFIX "")
target_link_libraries(${PROJECT_NAME}_rail_vis ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_stm32_control src/stm32_control.cpp)
set_target_properties(${PROJECT_NAME}_stm32_control PROPERTIES OUTPUT_NAME stm32_control PREFIX "")
target_link_libraries(${PROJECT_NAME}_stm32_control ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_stm32_vis src/stm32_visualisation.cpp)
set_target_properties(${PROJECT_NAME}_stm32_vis PROPERTIES OUTPUT_NAME stm32_vis PREFIX "")
target_link_libraries(${PROJECT_NAME}_stm32_vis ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_data_logger src/data_logger.cpp)
set_target_properties(${PROJECT_NAME}_data_logger PROPERTIES OUTPUT_NAME data_logger PREFIX "")
target_link_libraries(${PROJECT_NAME}_data_logger ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_simulation src/simulation.cpp)
set_target_properties(${PROJECT_NAME}_simulation PROPERTIES OUTPUT_NAME simulation PREFIX "")
target_link_libraries(${PROJECT_NAME}_simulation ${catkin_LIBRARIES})