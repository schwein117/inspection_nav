cmake_minimum_required(VERSION 3.0.2)
project(inspection_nav)

add_compile_options(-std=c++14 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  geometry_msgs
  message_generation
  move_base_msgs
  nav_msgs
  roscpp
  std_msgs
)
find_package(yaml-cpp REQUIRED)

add_service_files(
  FILES
  GimbalAction.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES inspection_nav_core
  CATKIN_DEPENDS actionlib geometry_msgs message_runtime move_base_msgs nav_msgs roscpp std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(inspection_nav_core
  src/goal_file_io.cpp
  src/goal_recorder.cpp
  src/goal_sender.cpp
)

add_dependencies(inspection_nav_core ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(inspection_nav_core ${catkin_LIBRARIES} yaml-cpp)

add_executable(goal_recorder_node src/record.cpp)
add_dependencies(goal_recorder_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(goal_recorder_node inspection_nav_core ${catkin_LIBRARIES})

add_executable(goal_sender_node src/send.cpp)
add_dependencies(goal_sender_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(goal_sender_node inspection_nav_core ${catkin_LIBRARIES})

add_executable(route_path_publisher_node src/route_path_publisher.cpp)
add_dependencies(route_path_publisher_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(route_path_publisher_node inspection_nav_core ${catkin_LIBRARIES})

install(TARGETS inspection_nav_core goal_recorder_node goal_sender_node route_path_publisher_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
