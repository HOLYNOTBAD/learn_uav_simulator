# 在文件顶部添加：
cmake_minimum_required(VERSION 3.0.2)
project(path_searching)

# 设置C++标准
add_compile_options(-std=c++17)

# 添加必要的系统依赖
find_package(OpenCV REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

# 添加必要的ROS依赖
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
  cv_bridge
  message_filters
  plan_env
)

# 设置catkin包信息
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES path_searching
  CATKIN_DEPENDS roscpp rospy std_msgs visualization_msgs cv_bridge message_filters plan_env
  DEPENDS EIGEN3 Boost OpenCV PCL
)

# 设置包含目录
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# 添加链接目录
link_directories(${PCL_LIBRARY_DIRS})

# 添加库定义
add_library(path_searching
  src/kinodynamic_astar.cpp
)

# 链接依赖
add_dependencies(path_searching ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(path_searching
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)