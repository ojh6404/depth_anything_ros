cmake_minimum_required(VERSION 3.0.2)
project(depth_anything_ros)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  jsk_data
)
# Find and include CUDA
find_package(CUDA REQUIRED)
# Find and include OpenCV
find_package(OpenCV REQUIRED)

# ------------------------------------------------------------------------------------
# Download models
# ------------------------------------------------------------------------------------
add_custom_target(${PROJECT_NAME}_install_trained_data ALL COMMAND python$ENV{ROS_PYTHON_VERSION} ${PROJECT_SOURCE_DIR}/scripts/install_trained_data.py)

# Add source files
set(SOURCES
	src/utils.cpp
	src/depth_anything.cpp
	src/depth_estimation_node.cpp
)

# Add headers
set(HEADERS
  include/utils.h
	include/depth_anything.h
)

include_directories(
  ${CUDA_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
)

# Set TensorRT path
# If TENSORRT_DIR env variable is not set, set it to default path
if(NOT DEFINED ENV{TENSORRT_DIR})
    message("TENSORRT_DIR is not set. Setting it to default path.")
    set(TENSORRT_DIR "$ENV{HOME}/tensorrt")
else()
    message("TENSORRT_DIR is set to $ENV{TENSORRT_DIR}")
    set(TENSORRT_DIR "$ENV{TENSORRT_DIR}")
endif()

# Include TensorRT
include_directories(${TENSORRT_DIR}/include)
link_directories(${TENSORRT_DIR}/lib)

# Read TensorRT version
file(STRINGS "${TENSORRT_DIR}/include/NvInferVersion.h" tensorrt_version REGEX "#define NV_TENSORRT_MAJOR +[0-9]+")
string(REGEX MATCH "[0-9]+" tensorrt_version_major ${tensorrt_version})

# Check TensorRT version and set library accordingly
if (tensorrt_version_major EQUAL 10)
    set(TENSORRT_LIBS nvinfer nvinfer_plugin nvonnxparser)
else()
    set(TENSORRT_LIBS nvinfer nvinfer_plugin nvparsers nvonnxparser)
endif()

catkin_package (CATKIN_DEPENDS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  jsk_data
  LIBRARIES {PROJECT_NAME}
)

# Add executable
add_executable(depth_estimation_node ${SOURCES} ${HEADERS})

# Link libraries
target_link_libraries(depth_estimation_node
    ${OpenCV_LIBS}
    ${CUDA_LIBRARIES}
    ${TENSORRT_LIBS}
    ${catkin_LIBRARIES}
)
