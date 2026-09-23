cmake_minimum_required(VERSION 3.8)
project(g1_controller)

# ============================================================
# C++ Standard
# ============================================================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ============================================================
# Dependencies
# ============================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)


# ============================================================
# System Dependencies
# ============================================================
find_package(Boost REQUIRED COMPONENTS program_options)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)


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
  /usr/local
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
list(APPEND CMAKE_BUILD_RPATH "${RPATHS}")
list(APPEND CMAKE_INSTALL_RPATH "${RPATHS}")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# ============================================================
# Include directories
# ============================================================
include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  /usr/local/include/ddscxx
  /usr/local/include/iceoryx/v2.0.2
  ${ORT_DIR}/include
  ${PROJECT_SOURCE_DIR}/thirdparty/cnpy
)
set(CNPY_SRC ${PROJECT_SOURCE_DIR}/thirdparty/cnpy/cnpy.cpp)

# ============================================================
# Sources
# ============================================================
file(GLOB_RECURSE ADD_SRC_LIST
  "${PROJECT_SOURCE_DIR}/src/*.cpp"
)
list(FILTER ADD_SRC_LIST EXCLUDE REGEX ".*src/main.cpp$")


# ============================================================
# Library target
# ============================================================
add_library(${PROJECT_NAME}_lib ${ADD_SRC_LIST})
add_library(cnpy ${CNPY_SRC})
target_link_libraries(${PROJECT_NAME}_lib cnpy z)  # cnpy depends on zlib
find_package(ZLIB REQUIRED)
target_link_libraries(cnpy ZLIB::ZLIB)

ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  grid_map_core
  grid_map_ros
  grid_map_msgs
  ament_index_cpp
  Boost
  yaml-cpp
)

target_link_libraries(${PROJECT_NAME}_lib
  Eigen3::Eigen
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

ament_target_dependencies(g1_controller
  rclcpp
)


# ============================================================
# Install rules
# ============================================================
install(TARGETS g1_controller ${PROJECT_NAME}_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME} 
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/sam3_perception.py scripts/sam3_python.sh scripts/create_chair_scene.py scripts/create_desk_scene.py
                 scripts/sam3_localization.py scripts/sam3_benchmark.py
                 scripts/codex_ros_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES scripts/codex_ros_core.py scripts/codex_rgbd.py DESTINATION lib/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_lib)
ament_export_dependencies(
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  grid_map_core
  grid_map_ros
  grid_map_msgs
)

option(G1_BUILD_MUJOCO_CAMERA "Build the camera-enabled Unitree MuJoCo simulator" OFF)
if(G1_BUILD_MUJOCO_CAMERA)
  add_subdirectory(simulator_camera)
endif()

ament_package()

# ScaleBFM ONNX integrity and standalone core (opt-in FSM).
find_package(OpenSSL REQUIRED)
target_link_libraries(${PROJECT_NAME}_lib OpenSSL::Crypto)
