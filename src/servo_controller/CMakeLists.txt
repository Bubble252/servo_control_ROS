cmake_minimum_required(VERSION 3.0.2)
project(servo_controller)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## 查找catkin依赖
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

## 消息定义
add_message_files(
  FILES
  ServoFeedback.msg
)

## 自动生成消息
generate_messages(
  DEPENDENCIES
  std_msgs
)

## 包含头文件路径（确保包含servo.h所在目录和SCServo.h所在目录）
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}/../../../../src    # 假设SCServo.h在这里
)

## 添加外部静态库
add_library(SCServo STATIC IMPORTED)
set_target_properties(SCServo PROPERTIES IMPORTED_LOCATION
  ${CMAKE_SOURCE_DIR}/../../../../src/libSCServo.a
)

## 编译 servo.cpp 成为一个静态库
add_library(servo_lib
  src/servo.cpp
)

## 编译你的节点，将servo_node.cpp和servo_lib链接
add_executable(servo_node
  src/servo_node.cpp
)

target_link_libraries(servo_node
  servo_lib
  SCServo
  ${catkin_LIBRARIES}
)

## 添加依赖以确保消息先构建
add_dependencies(servo_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

