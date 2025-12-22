cmake_minimum_required(VERSION 3.0.2)
project(g1_controller)

# ============================================================
# C++ Standard
# ============================================================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ============================================================
# Catkin
# ============================================================
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  roslib
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  tf2_ros
  message_generation
)

# ============================================================
# System Dependencies
# ============================================================
find_package(Boost REQUIRED COMPONENTS program_options)
find_package(yaml-cpp REQUIRED)

# ============================================================
# Custom Messages
# ============================================================
add_message_files(
  FILES
  LowCmd.msg
  MotorCmd.msg
  LowState.msg
  IMUState.msg
  MotorState.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

# ============================================================
# ONNX Runtime (GPU build)
# ============================================================
set(ORT_DIR "${PROJECT_SOURCE_DIR}/thirdparty/onnxruntime-linux-x64-gpu-1.22.0")

if (NOT EXISTS "${ORT_DIR}/lib/libonnxruntime.so.1.22.0")
  message(FATAL_ERROR "libonnxruntime.so.1.22.0 not found in ${ORT_DIR}/lib (GPU version required).")
endif()

set(CUDA_LIB_HINTS
  /usr/local/cuda/lib64
  /usr/lib/x86_64-linux-gnu
  /usr/lib/aarch64-linux-gnu
)

find_library(NVINFER_LIB
  NAMES nvinfer libnvinfer.so libnvinfer.so.10
  HINTS ${CUDA_LIB_HINTS}
)

find_library(CUDNN_LIB
  NAMES cudnn libcudnn.so libcudnn.so.9
  HINTS ${CUDA_LIB_HINTS}
)

# Runtime search paths
set(RPATHS "${ORT_DIR}/lib")

if (NVINFER_LIB)
  get_filename_component(NVINFER_DIR "${NVINFER_LIB}" DIRECTORY)
  list(APPEND RPATHS "${NVINFER_DIR}")
else()
  message(WARNING "TensorRT runtime not found; TRT EP will fall back to CUDA.")
endif()

if (CUDNN_LIB)
  get_filename_component(CUDNN_DIR "${CUDNN_LIB}" DIRECTORY)
  list(APPEND RPATHS "${CUDNN_DIR}")
else()
  message(WARNING "cuDNN runtime not found; CUDA EP may fall back to CPU.")
endif()

list(REMOVE_DUPLICATES RPATHS)
set(CMAKE_BUILD_RPATH "${RPATHS}")
set(CMAKE_INSTALL_RPATH "${RPATHS}")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# ============================================================
# Catkin export: public headers + libs
# ============================================================
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES g1_controller_lib
  CATKIN_DEPENDS roscpp std_msgs roslib geometry_msgs nav_msgs sensor_msgs visualization_msgs tf2_ros message_runtime
)

# ============================================================
# Include directories
# ============================================================
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  /usr/include/eigen3
  /usr/local/include/ddscxx
  /usr/local/include/iceoryx/v2.0.2
  ${ORT_DIR}/include
)

# ============================================================
# Sources
# ============================================================
file(GLOB_RECURSE ADD_SRC_LIST
  ${PROJECT_SOURCE_DIR}/src/*.cpp
)

# ============================================================
# Library target
# ============================================================
add_library(${PROJECT_NAME}_lib ${ADD_SRC_LIST})

target_link_libraries(${PROJECT_NAME}_lib
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  yaml-cpp
  unitree_sdk2
  ddsc
  ddscxx
  fmt
  rt
  pthread
  ${ORT_DIR}/lib/libonnxruntime.so.1.22.0
)

if (NVINFER_LIB)
  target_link_libraries(${PROJECT_NAME}_lib
  ${NVINFER_LIB}
  )
endif()

if (CUDNN_LIB)
  target_link_libraries(${PROJECT_NAME}_lib
  ${CUDNN_LIB}
  )
endif()

# ============================================================
# Executable
# ============================================================
add_executable(g1_controller src/main.cpp)

target_link_libraries(g1_controller
  ${PROJECT_NAME}_lib
)

# ============================================================
# Install rules
# ============================================================
install(TARGETS g1_controller ${PROJECT_NAME}_lib
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
