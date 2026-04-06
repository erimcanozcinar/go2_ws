cmake_minimum_required(VERSION 3.0.2)
project(third_party)

# Find catkin so it can be built with catkin_make
find_package(catkin REQUIRED)

# Declare the catkin package, exporting the optimizer headers and libraries 
# so other ROS packages in your workspace can find and link against them
catkin_package(
  INCLUDE_DIRS Goldfarb_Optimizer
  LIBRARIES Goldfarb_Optimizer
)

# Build the Goldfarb_Optimizer subdirectory
add_subdirectory(Goldfarb_Optimizer)