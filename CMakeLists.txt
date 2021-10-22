cmake_minimum_required(VERSION 3.0.2)
project(evarobot_adp)
set(CMAKE_PREFIX_PATH _PATH "/opt/ros/noetic;${CMAKE_MODULE_PATH}")
find_package(PCL 1.10)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  image_transport
  image_geometry
  cv_bridge
  geometry_msgs
  message_generation
  laser_geometry
)
add_message_files(
    DIRECTORY msg
    FILES
    ObjectList.msg
    Object.msg
    )
generate_messages(DEPENDENCIES std_msgs geometry_msgs)
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs
)
include_directories(
  include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS}
)

add_library(evarobot_process_cam src/evarobot_process_cam.cpp)
  add_dependencies(evarobot_process_cam ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(evarobot_process_cam
    ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES}
  )

add_library(evarobot_process_lid src/evarobot_process_lid.cpp)
  add_dependencies(evarobot_process_lid ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(evarobot_process_lid
    ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES}
  )
  
add_library(evarobot_process_sonar src/evarobot_process_sonar.cpp)
  add_dependencies(evarobot_process_sonar ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(evarobot_process_sonar
    ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES}
  )
add_library(kalman_filter src/kalman_filter.cpp)
  add_dependencies(kalman_filter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(kalman_filter
    ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES}
  )
add_executable(evarobot_process_node src/evarobot_process_node.cpp)
  add_dependencies(evarobot_process_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(evarobot_process_node
    ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES}
  )
