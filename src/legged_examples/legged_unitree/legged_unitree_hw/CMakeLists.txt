cmake_minimum_required(VERSION 3.10)
project(legged_unitree_hw)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        legged_hw
)

catkin_package(
        INCLUDE_DIRS
        include
        CATKIN_DEPENDS
        roscpp
        legged_hw
)

## Import Unitree SDK2 library
set(UNITREE_SDK_PATH /usr/local/lib)
find_library(UNITREE_SDK_LIB unitree_sdk2 PATHS ${UNITREE_SDK_PATH} NO_DEFAULT_PATH)

if (NOT UNITREE_SDK_LIB)
    message(FATAL_ERROR "Unitree SDK library for the architecture is not found")
else ()
    message(STATUS "Unitree SDK library found at: ${UNITREE_SDK_LIB}")
endif ()

message(STATUS "Importing: ${UNITREE_SDK_LIB}")

find_package(Threads REQUIRED)

add_library(unitree_sdk2 STATIC IMPORTED GLOBAL)
set_target_properties(unitree_sdk2 PROPERTIES IMPORTED_LOCATION ${UNITREE_SDK_LIB})
target_link_libraries(unitree_sdk2 INTERFACE ddsc ddscxx Threads::Threads)
target_include_directories(unitree_sdk2 INTERFACE $<BUILD_INTERFACE:/usr/local/include> $<INSTALL_INTERFACE:include>)

###########
## Build ##
###########

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        /usr/local/include/ddscxx
)

link_directories(lib)

set(SOURCE src/legged_unitree_hw.cpp src/UnitreeHW.cpp)

add_executable(${PROJECT_NAME}_3_3_1
        ${SOURCE}
        )

target_compile_definitions(${PROJECT_NAME}_3_3_1 PRIVATE UNITREE_SDK_3_3_1)

target_link_libraries(${PROJECT_NAME}_3_3_1
        ${catkin_LIBRARIES}
        libunitree_legged_sdk_amd64_3_3_1.so lcm
        )

add_executable(${PROJECT_NAME}_3_8_0
        ${SOURCE}
)

target_compile_definitions(${PROJECT_NAME}_3_8_0 PRIVATE UNITREE_SDK_3_8_0)

target_link_libraries(${PROJECT_NAME}_3_8_0
        ${catkin_LIBRARIES}
        libunitree_legged_sdk_amd64_3_8_0.a
)

add_executable(${PROJECT_NAME}_go2
        src/legged_unitree_hw_go2.cpp 
        src/UnitreeGo2HW.cpp
)

target_link_libraries(${PROJECT_NAME}_go2
        ${catkin_LIBRARIES}
        unitree_sdk2
)

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND)
    message(STATUS "Run clang tooling for target " ${PROJECT_NAME})
    add_clang_tooling(
            TARGETS ${PROJECT_NAME}_3_3_1 TARGETS ${PROJECT_NAME}_3_8_0
            SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include/legged_unitree_hw
            CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include/legged_unitree_hw
            CF_WERROR
    )

endif (cmake_clang_tools_FOUND)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}_3_3_1 ${PROJECT_NAME}_3_8_0
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}_3_3_1/ include/${PROJECT_NAME}_3_8_0/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        )

# Mark other files for installation
install(DIRECTORY config launch lib
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )
