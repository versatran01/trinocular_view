cmake_minimum_required(VERSION 2.8.3)
project(snapshot)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  )

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  )

add_executable(${PROJECT_NAME}_node
  src/snapshot_node.cpp
  )
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  )

add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
