cmake_minimum_required(VERSION 2.8.3)
project(speed_test)

set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O4 -Wall" )

find_package(catkin REQUIRED COMPONENTS 
	roscpp
	std_msgs
	message_generation
)

add_message_files(
    FILES
	comm.msg
    control.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(CATKIN_DEPENDS
	roscpp
	std_msgs
	message_generation
)

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})

add_executable(calculation_speed_exe src/calculation.cpp)
target_link_libraries(calculation_speed_exe ${catkin_LIBRARIES})
add_dependencies(calculation_speed_exe ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(controller_speed_exe src/controller.cpp)
target_link_libraries(controller_speed_exe ${catkin_LIBRARIES})
add_dependencies(controller_speed_exe ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
