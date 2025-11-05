cmake_minimum_required(VERSION 3.0.2)
project(log_replay)

set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        pcl_ros
        std_msgs
        nav_msgs
        cv_bridge
        sensor_msgs
        geometry_msgs
        image_transport
        jsk_recognition_msgs)

catkin_package(
        CATKIN_DEPENDS roscpp std_msgs nav_msgs sensor_msgs geometry_msgs
)

include_directories(include
        ${catkin_INCLUDE_DIRS})
link_directories(${PATH_THIRDPARTY}/perceptor/lib)
file(GLOB_RECURSE SRC src/*.cpp)

set(SRC
        src/ClassifySim.cpp
        src/LogReplay.cpp
        src/TofMergerSim.cpp
        src/WireDetectionSim.cpp
        src/Mapping3dSim.cpp
        src/InvasionSim.cpp
        src/PostFusionSim.cpp
        )

# code may not exist in perception.tar.gz, so may not need to compile related function
if (NOT CP_POSTFUSION_NODE_OPTION)
    list(REMOVE_ITEM SRC "src/PostFusionSim.cpp")
endif ()

if (NOT CP_INVASION_NODE_OPTION)
    list(REMOVE_ITEM SRC "src/InvasionSim.cpp")
endif ()

add_library(${PROJECT_NAME} SHARED ${SRC})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_definitions(-DROS_AVAILABLE=1)
add_definitions(-DSOURCE_DIR="${CMAKE_SOURCE_DIR}")

option(ENABLE_MASK "Enable mask" OFF)

if (ENABLE_MASK)
    add_definitions(-DENABLE_MASK=1)
else ()
    add_definitions(-DENABLE_MASK=0)
endif ()

target_include_directories(${PROJECT_NAME} PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/include
        ${PATH_LOGPARSE}/include
        ${PATH_THIRDPARTY}/perceptor/include
        ${PCL_INCLUDE_DIRS}
        )
target_link_libraries(${PROJECT_NAME} PUBLIC ${catkin_LIBRARIES} ${PCL_LIBRARIES} yaml-cpp)
target_link_libraries(${PROJECT_NAME} PUBLIC log_parser perceptor stdc++fs pthread)
# lib perceptor use openmp, log_replay must link, otherwise cannot build
target_link_libraries(${PROJECT_NAME} PUBLIC OpenMP::OpenMP_CXX)

if (GLMEASURE_ENABLED)
    target_link_libraries(${PROJECT_NAME} PUBLIC EGL GLESv2)
endif ()

if (TARGET dbow3)
    target_link_libraries(${PROJECT_NAME} PUBLIC dbow3)
endif ()

if (TARGET eigen)
    target_link_libraries(${PROJECT_NAME} PUBLIC eigen)
endif ()

## see here: https://github.com/PointCloudLibrary/pcl/issues/3680
## when this is fixed, we can remove the following 3 lines.
if (NOT DEFINED CMAKE_SUPPRESS_DEVELOPER_WARNINGS)
    set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "No dev warnings")
endif ()

if (CP_REPLAY_NODE_OPTION)
    add_executable(replay node/log_replay_node.cpp)
    target_link_libraries(replay PUBLIC ${PROJECT_NAME})
endif ()

if (CP_TOFMERGER_NODE_OPTION)
    add_executable(tof_merger node/tof_merger_node.cpp)
    target_link_libraries(tof_merger PUBLIC ${PROJECT_NAME})
endif ()

if (CP_CLASSIFY_NODE_OPTION)
    add_executable(classify node/classify_node.cpp)
    target_link_libraries(classify PUBLIC ${PROJECT_NAME})
endif ()

if (CP_MAPPING3D_NODE_OPTION)
    add_executable(mapping3d node/mapping3d_node.cpp)
    target_link_libraries(mapping3d PUBLIC ${PROJECT_NAME})
endif ()

if (CP_WIREDETECT_NODE_OPTION)
    add_executable(wire_detection node/wire_detection_node.cpp)
    target_link_libraries(wire_detection PUBLIC ${PROJECT_NAME})
endif ()

if (CP_INVASION_NODE_OPTION)
    add_executable(invasion_detection node/invasion_detection_node.cpp)
    target_link_libraries(invasion_detection PUBLIC ${PROJECT_NAME})
endif ()

if (CP_POSTFUSION_NODE_OPTION)
    add_executable(post_fusion node/post_fusion_node.cpp)
    target_link_libraries(post_fusion PUBLIC ${PROJECT_NAME})
endif ()

SET(CMAKE_INSTALL_PREFIX ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS
        ${PROJECT_NAME}
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Mark cpp header files for installation
install(DIRECTORY include/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        PATTERN ".svn" EXCLUDE)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch rviz scripts
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
