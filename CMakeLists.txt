cmake_minimum_required(VERSION 3.8)
project(pgo_r2h)



# ----------------------------------------------------------------------------------------------- #

# default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# compile option
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ----------------------------------------------------------------------------------------------- #



#
# find package
find_package(ament_cmake REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(GTSAM REQUIRED)
find_package(OpenMP REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)



#
# include
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
)



#
# +@ for PCL
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})



# ----------------------------------------------------------------------------------------------- #

#
# target < node_pgo >
add_executable(node_pgo
  src/node_pgo.cpp
  src/pgo.cpp
  src/lib/conversion.cpp
  src/lib/ros2.cpp
  src/lib/time.cpp
)
target_link_libraries(node_pgo
  Eigen3::Eigen
  ${PCL_LIBRARIES}
  gtsam
  OpenMP::OpenMP_CXX
)
ament_target_dependencies(node_pgo
  rclcpp
  tf2
  pcl_conversions
  cv_bridge
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
)

#
# install targets
install(TARGETS
  node_pgo
  DESTINATION lib/${PROJECT_NAME}
)

# ----------------------------------------------------------------------------------------------- #



#
# install directories
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)



#
# ament package
ament_package()
