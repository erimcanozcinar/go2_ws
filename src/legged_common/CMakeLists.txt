cmake_minimum_required(VERSION 3.10)
project(legged_common)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        sensor_msgs
        hardware_interface
        )

find_package(SDL2 REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
        INCLUDE_DIRS
        include
        LIBRARIES
        ${PROJECT_NAME}
        CATKIN_DEPENDS
        roscpp
        hardware_interface
        DEPENDS
)

include_directories(
        include
        "/usr/include/eigen3"
        ${catkin_INCLUDE_DIRS}
        ${Eigen3_INCLUDE_DIRS}
        ${SDL2_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
        src/kinematics.cpp
        src/matTools.cpp
        src/GamePad.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC include)

add_executable(gamepad_node src/GamePad_node.cpp)
target_link_libraries(gamepad_node ${catkin_LIBRARIES} SDL2)

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND)
    message(STATUS "Run clang tooling for target " ${PROJECT_NAME})
    add_clang_tooling(
            TARGETS ${PROJECT_NAME}
            SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
            CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
            CF_WERROR
    )
endif (cmake_clang_tools_FOUND)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(
        TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(
        DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
)

install(PROGRAMS scripts/generate_urdf.sh
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/scripts
        )

