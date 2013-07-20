cmake_minimum_required(VERSION 2.8.3)
project(moveit_web_robot_state)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Eigen REQUIRED)
find_package(Boost REQUIRED thread date_time system filesystem program_options python)
find_package(PythonLibs)
find_package(catkin REQUIRED COMPONENTS
  moveit_ros_planning_interface
  moveit_ros_planning
  moveit_ros_manipulation
  moveit_ros_move_group
  eigen_conversions
  tf
  roscpp
  actionlib
  rospy
  rosconsole
)

catkin_python_setup()

catkin_package(
  LIBRARIES  
    moveit_web_robot_state
  CATKIN_DEPENDS
    moveit_ros_planning_interface
    moveit_ros_planning
    moveit_ros_manipulation
    moveit_ros_move_group
)

include_directories(${catkin_INCLUDE_DIRS})

include_directories(SYSTEM
                    ${EIGEN_INCLUDE_DIRS}
		                ${Boost_INCLUDE_DIRS}
                    ${PYTHON_INCLUDE_DIRS})
	
link_directories(${Boost_LIBRARY_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_library(moveit_web_robot_state_python src/wrap_python_robot_state.cpp)
target_link_libraries(moveit_web_robot_state_python ${PYTHON_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES} moveit_py_bindings_tools moveit_move_group_interface)
set_target_properties(moveit_web_robot_state_python PROPERTIES OUTPUT_NAME moveit_web_robot_state PREFIX "")
set_target_properties(moveit_web_robot_state_python PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION})

install(TARGETS moveit_web_robot_state_python
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION})
