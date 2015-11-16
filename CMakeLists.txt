cmake_minimum_required(VERSION 2.8.3)
project(snapshot)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  image_transport
  message_filters
  cv_bridge
  )

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  )

add_executable(${PROJECT_NAME}
  src/trinocular_view.cpp
  )
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  )

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
