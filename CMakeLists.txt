cmake_minimum_required(VERSION 2.8.3)
project(kinect_filter)

find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)
find_package(actionlib REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL 1.7 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_ros
  pcl_conversions
)

link_directories(${PCL_LIBRARY_DIRS})

catkin_package(
  INCLUDE_DIRS include

  CATKIN_DEPENDS roscpp sensor_msgs pcl_ros pcl_conversions

  DEPENDS eigen system_lib 
)

include_directories(include)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_NAME}/include
  ${Eigen_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_definitions(${EIGEN_DEFINITIONS})
add_definitions(${PCL_DEFINITIONS})

# ----------

add_executable(characterize_noise src/characterize_noise.cpp)

add_dependencies(characterize_noise ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(characterize_noise
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
