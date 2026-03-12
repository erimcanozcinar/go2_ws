cmake_minimum_required(VERSION 3.10)
project(legged_controllers)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#link_directories("/usr/local/lib")

## Find catkin macros and libraries
find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        legged_common
        #legged_interface
        #legged_wbc
        legged_estimator
        legged_trajectory
        controller_interface
        #ocs2_legged_robot_ros
        #ocs2_self_collision_visualization
        #angles
        realtime_tools
        std_msgs
        message_generation
        qpoases_catkin
)

find_package(SDL2 REQUIRED)
find_package(rbdyn REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories("/usr/include/eigen3")
find_package(PkgConfig REQUIRED)
#pkg_check_modules(pinocchio REQUIRED pinocchio)

#add_service_files(
#  FILES
#  GetBodyHeight.srv
#)

#generate_messages(DEPENDENCIES std_msgs)

catkin_package(
        INCLUDE_DIRS
        include
        LIBRARIES
        ${PROJECT_NAME}
        CATKIN_DEPENDS
        roscpp
        legged_common
        #legged_interface
        #legged_wbc
        legged_estimator
        legged_trajectory
        controller_interface
        #ocs2_legged_robot_ros
        #ocs2_self_collision_visualization
        #angles
        qpoases_catkin
)

###########
## Build ##
###########

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${pinocchio_INCLUDE_DIRS}
        ${Eigen3_INCLUDE_DIRS}
        ${SDL2_INCLUDE_DIRS}
        # ${rbdyn_INCLUDE_DIRS}
)

link_directories(
        ${pinocchio_LIBRARY_DIRS}
)

## Declare a cpp library
add_library(${PROJECT_NAME}
        src/Go2SquatController.cpp
        src/LeggedGo2TabletController.cpp
        src/Go2BalanceController.cpp
        src/Go2MPC.cpp
        src/Go2WBC.cpp
        src/BalanceController/BalanceController.cpp
        src/MPC/MPC.cpp
        src/WBC/WBC.cpp
        src/WBC/TaskSet/BodyOriTask.cpp
        src/WBC/TaskSet/BodyPosTask.cpp
        src/WBC/TaskSet/FootPosTask.cpp
        src/WBC/TaskSet/SingleContactTask.cpp      
)

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${pinocchio_LIBRARIES}
        ${SDL2_LIBRARIES}
        rbdyn::rbdyn
        )

target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})


# Target trajectories publish node
#add_executable(get_body_height_server src/GetBodyHeightServer.cpp)
#target_link_libraries(get_body_height_server ${catkin_LIBRARIES})

#add_executable(service_test src/service_test.cpp)
#target_link_libraries(service_test ${catkin_LIBRARIES})

#add_executable(legged_target_trajectories_publisher
#        src/TargetTrajectoriesPublisher.cpp
#        )
#add_dependencies(legged_target_trajectories_publisher
#        ${PROJECT_NAME}
#        ${catkin_EXPORTED_TARGETS}
#        )
#target_link_libraries(legged_target_trajectories_publisher
#        ${PROJECT_NAME}
#        ${catkin_LIBRARIES}
#        )
#target_compile_options(legged_target_trajectories_publisher PRIVATE ${OCS2_CXX_FLAGS})

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND)
    message(STATUS "Run clang tooling for target " ${PROJECT_NAME})
    add_clang_tooling(
            TARGETS ${PROJECT_NAME} legged_target_trajectories_publisher
            SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
            CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
            CF_WERROR
    )
endif (cmake_clang_tools_FOUND)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        )

# Install plugins
install(FILES legged_controllers_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )
