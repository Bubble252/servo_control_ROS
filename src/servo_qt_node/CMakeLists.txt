cmake_minimum_required(VERSION 3.0.2)
project(servo_qt_node)

# 基础配置
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)       # 自动处理Q_OBJECT
set(CMAKE_AUTOUIC ON)       # 自动处理UI文件
set(CMAKE_AUTORCC ON)       # 自动处理资源文件
set(CMAKE_INCLUDE_CURRENT_DIR ON)
add_compile_options(-Wall -Wextra -Wpedantic)

# 查找ROS依赖
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_runtime
  servo_controller
)

# 查找Qt依赖
find_package(Qt5 REQUIRED COMPONENTS Widgets Charts Core)

# 声明catkin包信息
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs message_runtime servo_controller
  INCLUDE_DIRS include  # 声明明确clude目录包含头文件
)

# 包含目录
include_directories(
  include  # 头文件根目录
  include/servo_qt_node  # 显式包含头文件所在的子目录
  ${catkin_INCLUDE_DIRS}
)

# 声明包含Q_OBJECT的头文件（完全匹配你的路径）
set(MOC_HEADERS
  include/servo_qt_node/qt_servo_control.h
)

# 处理UI文件（匹配你的ui目录路径）
qt5_wrap_ui(UISrcs ${CMAKE_CURRENT_SOURCE_DIR}/ui/qt_servo_control.ui)

# 声明可执行文件及源文件
add_executable(qt_servo_control_node
  src/main.cpp
  src/qt_servo_control.cpp
  ${UISrcs}
  ${MOC_HEADERS}
)

# 链接库
target_link_libraries(qt_servo_control_node
  ${catkin_LIBRARIES}
  Qt5::Widgets
  Qt5::Charts
  Qt5::Core
)

# 辅助moc识别头文件
target_sources(qt_servo_control_node PRIVATE ${MOC_HEADERS})

# 依赖关系
add_dependencies(qt_servo_control_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# 安装可执行文件（可选，便于系统调用）
install(TARGETS qt_servo_control_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
    
