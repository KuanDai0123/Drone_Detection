cmake_minimum_required(VERSION 3.0.2)
project(panorama)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  dvs_msgs
  sensor_msgs
  image_geometry
  image_transport
  cv_bridge
)

find_package(OpenCV REQUIRED)
find_package(OpenCL)
# find_package(PCL)

include_directories(/usr/include/eigen3)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES panorama
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)
if (NOT OPENCL_FOUND)
    set(OpenCL_LIBRARY "")
    set(OpenCL_INCLUDE_DIRS "")
endif (NOT OPENCL_FOUND)

add_definitions(-DOPENCL_ENABLED=${OPENCL_FOUND})

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include  # 添加项目自身的include目录
    ${catkin_INCLUDE_DIRS}
		${OpenCV_LIBS}
	  ${OpenCL_LIBRARY}
    ${catkin_INCLUDE_DIRS}
    /usr/include/glog
    )

catkin_install_python(PROGRAMS scripts/publish_camera_info.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(Panorama 
    src/panorama.cpp
    src/node.cpp
    src/frontend/ang_vel_estimator.cpp 
    src/backend/pose_graph_optimizer.cpp)


target_link_libraries(Panorama 
		${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${cv_bridge_LIBRARIES}
    # image_transport
    -lglog
    gflags
)
add_dependencies(Panorama ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

