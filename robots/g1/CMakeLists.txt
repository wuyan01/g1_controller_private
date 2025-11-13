cmake_minimum_required(VERSION 3.12)
project(g1_controller)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ============================================================
# System Dependencies
# ============================================================
find_package(Boost REQUIRED COMPONENTS program_options)
find_package(yaml-cpp REQUIRED)

# ---- ONNX Runtime (GPU build) ----
set(ORT_DIR "${PROJECT_SOURCE_DIR}/../../thirdparty/onnxruntime-linux-x64-gpu-1.22.0")
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
# Include directories
# ============================================================
include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  /usr/include/eigen3
  /usr/local/include/ddscxx
  /usr/local/include/iceoryx/v2.0.2
  ${ORT_DIR}/include
  ${PROJECT_SOURCE_DIR}/../../include/
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
  ${Boost_LIBRARIES}
  yaml-cpp
  unitree_sdk2
  ddsc
  ddscxx
  fmt
  rt
  pthread
  ${ORT_DIR}/lib/libonnxruntime.so.1.22.0
  ${NVINFER_LIB}
  ${CUDNN_LIB}
)

# ============================================================
# Executable
# ============================================================
add_executable(g1_ctrl main.cpp)

target_link_libraries(g1_ctrl
  ${PROJECT_NAME}_lib
)
