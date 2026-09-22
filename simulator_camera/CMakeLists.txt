cmake_minimum_required(VERSION 3.16)
project(g1_mujoco_camera LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
# The controller exports directory-wide grid_map Eigen plugins and a different
# param.h. Keep this vendor executable independent of those inherited settings.
set_directory_properties(PROPERTIES INCLUDE_DIRECTORIES "" COMPILE_DEFINITIONS "")
set(UNITREE_MUJOCO_DIR "" CACHE PATH "Path to the unitree_mujoco checkout")
if(NOT EXISTS "${UNITREE_MUJOCO_DIR}/simulate/src/main.cc")
  message(FATAL_ERROR "Set UNITREE_MUJOCO_DIR to your unitree_mujoco checkout")
endif()
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options)
list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics/lib/cmake")
find_package(unitree_sdk2 REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
set(SIM "${UNITREE_MUJOCO_DIR}/simulate")
execute_process(
  COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/prepare_sources.py"
          "${SIM}" "${CMAKE_CURRENT_BINARY_DIR}"
  RESULT_VARIABLE prepare_result)
if(NOT prepare_result EQUAL 0)
  message(FATAL_ERROR "Unsupported simulator source layout; see prepare_sources.py error")
endif()
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
  "${SIM}/src/main.cc" "${SIM}/mujoco/simulate/simulate.cc"
  "${CMAKE_CURRENT_SOURCE_DIR}/prepare_sources.py")
file(GLOB UI_SOURCES "${SIM}/mujoco/simulate/glfw_*.cc"
                    "${SIM}/mujoco/simulate/platform_*.cc")
add_executable(g1_mujoco_camera
  "${CMAKE_CURRENT_BINARY_DIR}/main.cc"
  "${CMAKE_CURRENT_BINARY_DIR}/simulate.cc"
  camera_publisher.cpp ${UI_SOURCES}
  "${SIM}/src/joystick/joystick.cc" "${SIM}/src/lodepng/lodepng.cpp")
target_include_directories(g1_mujoco_camera PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}" "${SIM}/src" "${SIM}/src/lodepng"
  "${SIM}/mujoco/include" "${SIM}/mujoco/simulate")
target_link_directories(g1_mujoco_camera PRIVATE "${SIM}/mujoco/lib")
target_link_libraries(g1_mujoco_camera mujoco glfw yaml-cpp unitree_sdk2
                      Boost::program_options fmt pthread)
target_compile_options(g1_mujoco_camera PRIVATE "$<$<NOT:$<CONFIG:Debug>>:-O3>")
ament_target_dependencies(g1_mujoco_camera rclcpp sensor_msgs visualization_msgs tf2_ros tf2)
set_target_properties(g1_mujoco_camera PROPERTIES
  INSTALL_RPATH "${SIM}/mujoco/lib;/opt/unitree_robotics/lib")
install(TARGETS g1_mujoco_camera RUNTIME DESTINATION lib/g1_controller)
