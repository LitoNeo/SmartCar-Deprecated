cmake_minimum_required(VERSION 2.8.3)
project(test_ndt_matching)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(PCL 1.7 REQUIRED)
find_package(CUDA)
if (CUDA_FOUND)
	add_definitions(-DCUDA_FOUND)
	include_directories(${CUDA_INCLUDE_DIRS})
endif ()
include_directories(include)
link_directories(lib)
find_package(Eigen3 REQUIRED)
# find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	config_msgs
  nav_msgs
  pcl_conversions
  sensor_msgs
	tf
	std_msgs
	geometry_msgs
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES fast_ndt_slam
  CATKIN_DEPENDS nav_msgs roscpp tf sensor_msgs std_msgs geometry_msgs pcl_conversions config_msgs
)


include_directories(
  include ${catkin_INCLUDE_DIRS}
  include/
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)


add_executable(test_ndt_matching_node src/test_ndt_matching_node.cpp src/test_ndt_matching.cpp include/fast_ndt_matching/test_ndt_matching.h)
target_link_libraries(test_ndt_matching_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(test_ndt_matching_node ndt_cpu pcl_omp_registration)
add_dependencies(test_ndt_matching_node config_msgs)
if (CUDA_FOUND)
	target_link_libraries(test_ndt_matching_node ndt_gpu)
endif ()
# add_dependencies(fast_ndt_mapping_node ${catkin_EXPORTED_TARGETS} )

# 正确顺序  --错误顺序将导致编译失败!!!!!
#find_package()
#catkin_package()
#include_directories()
#
#add_executable()
#add_library()
#target_link_libraries()
