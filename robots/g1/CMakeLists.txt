cmake_minimum_required(VERSION 3.12)
project(g1_controller)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---- ONNX Runtime (GPU build) ----
set(ORT_DIR "${PROJECT_SOURCE_DIR}/../../thirdparty/onnxruntime-linux-x64-gpu-1.22.0")
if (NOT EXISTS "${ORT_DIR}/lib/libonnxruntime.so.1.22.0")
  message(FATAL_ERROR "libonnxruntime.so.1.22.0 not found in ${ORT_DIR}/lib. Make sure you used the *GPU* package.")
endif()

set(CUDA_LIB_HINTS
  /usr/local/cuda/lib64
  /usr/lib/x86_64-linux-gnu
  /usr/lib/aarch64-linux-gnu
)

# Accept both dev (unversioned) and runtime (versioned) names
find_library(NVINFER_LIB
  NAMES nvinfer libnvinfer.so libnvinfer.so.10
  HINTS ${CUDA_LIB_HINTS}
)

find_library(CUDNN_LIB
  NAMES cudnn libcudnn.so libcudnn.so.9
  HINTS ${CUDA_LIB_HINTS}
)

# RPATHS always include ORT lib; add NVINFER/CUDNN dirs if found
set(RPATHS "${ORT_DIR}/lib")
if (NVINFER_LIB)
  get_filename_component(NVINFER_DIR "${NVINFER_LIB}" DIRECTORY)
  list(APPEND RPATHS "${NVINFER_DIR}")
endif()
if (CUDNN_LIB)
  get_filename_component(CUDNN_DIR "${CUDNN_LIB}" DIRECTORY)
  list(APPEND RPATHS "${CUDNN_DIR}")
endif()
list(REMOVE_DUPLICATES RPATHS)

set(CMAKE_BUILD_RPATH "${RPATHS}")
set(CMAKE_INSTALL_RPATH "${RPATHS}")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# Make the message reflect runtime-vs-dev correctly
if (NOT NVINFER_LIB)
  message(WARNING "TensorRT runtime not found on search path; TRT EP will fall back to CUDA.")
endif()
if (NOT CUDNN_LIB)
  message(WARNING "cuDNN runtime not found on search path; CUDA EP will fall back to CPU.")
endif()


# ---- Dependencies ----
find_package(Boost REQUIRED COMPONENTS program_options)
find_package(yaml-cpp REQUIRED)

include_directories(
  /usr/include/eigen3
  /usr/local/include/ddscxx
  /usr/local/include/iceoryx/v2.0.2
  include
  ${PROJECT_SOURCE_DIR}/../../thirdparty/ocs2/include
  ${ORT_DIR}/include
  ${PROJECT_SOURCE_DIR}/../../include/
)

link_libraries(
  unitree_sdk2 ddsc ddscxx rt pthread # dds
  libboost_program_options.a libyaml-cpp.a fmt
  ${ORT_DIR}/lib/libonnxruntime.so.1.22.0
)

file(GLOB_RECURSE ADD_SRC_LIST
  ${PROJECT_SOURCE_DIR}/src/*.cpp
)

add_library(${PROJECT_NAME}_lib ${ADD_SRC_LIST})
link_libraries(${PROJECT_NAME}_lib)

add_executable(g1_ctrl main.cpp)