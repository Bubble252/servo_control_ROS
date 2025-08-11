cmake_minimum_required(VERSION 3.0.2)
project(servo_controller)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

add_message_files(
  FILES
  ServoFeedback.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(   # <<<<<<<<<<<✅ 这一段是你漏掉的
  INCLUDE_DIRS include
  LIBRARIES servo_lib
  CATKIN_DEPENDS roscpp std_msgs message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}/../../../../src
)

add_library(SCServo STATIC IMPORTED)
set_target_properties(SCServo PROPERTIES IMPORTED_LOCATION
  ${CMAKE_SOURCE_DIR}/../../../../src/libSCServo.a
)

add_library(servo_lib
  src/servo.cpp
)

add_executable(servo_node
  src/servo_node.cpp
)

target_link_libraries(servo_node
  servo_lib
  SCServo
  ${catkin_LIBRARIES}
)

add_dependencies(servo_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

