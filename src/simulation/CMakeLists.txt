cmake_minimum_required(VERSION 3.8)
project(simulation)
set(CMAKE_PREFIX_PATH "/home/erim/raisim_ws_v1.1.7/install")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(raisim CONFIG REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/include)
link_directories("/usr/local/lib")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_executable(sim src/main_sim.cpp src/simulationBridge.cpp)
ament_target_dependencies(sim rclcpp sensor_msgs)
target_link_libraries(sim raisim::raisim pthread)

install(TARGETS
  sim
  DESTINATION lib/${PROJECT_NAME})


ament_package()
