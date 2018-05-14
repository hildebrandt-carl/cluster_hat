cmake_minimum_required(VERSION 2.8.3)
project(image_processing_ros)

set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O4 -Wall" )

find_package(catkin REQUIRED COMPONENTS 
	roscpp
	std_msgs
	message_generation
	cv_bridge
	image_transport
	sensor_msgs
)

find_package(OpenCV REQUIRED)

add_message_files(
    FILES
	comm.msg
    control.msg
	split_img.msg
	return_img.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package(CATKIN_DEPENDS
	roscpp
	std_msgs
	message_generation
)

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})

add_executable(processor_imgpro_exe src/processor.cpp)
target_link_libraries(processor_imgpro_exe ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(processor_imgpro_exe ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(controller_imgpro_exe src/controller.cpp)
target_link_libraries(controller_imgpro_exe ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(controller_imgpro_exe ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})