cmake_minimum_required(VERSION 3.0)

project(log_terminal)

set(MODEL_LIST_DIR ${CMAKE_SOURCE_DIR}/tools/third_party/wire_detection)

message("log terminal MODEL_LIST_DIR: ${MODEL_LIST_DIR}")
# set(WORK_SPACE_LOCATION ${CMAKE_SOURCE_DIR})
# configure_file(global_definition.h.in include/global_definition.h)
include_directories(${PROJECT_BINARY_DIR}/include)

add_definitions(-DSOURCE_DIR="${CMAKE_SOURCE_DIR}")

option(ENABLE_MASK "Enable mask" OFF)

if (ENABLE_MASK)
    add_definitions(-DENABLE_MASK=1)   
else()
    add_definitions(-DENABLE_MASK=0) 
endif()
find_package(OpenCV REQUIRED)
link_directories(${PATH_THIRDPARTY}/perceptor/lib)

if(CP_MAPPING3D_NODE_OPTION)
    add_executable(run_mapping_3d run_mapping3d.cpp)
    target_include_directories(run_mapping_3d PUBLIC ${PATH_LOGREPLAY}/include ${PATH_THIRDPARTY}/perceptor/include ${EIGEN3_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(run_mapping_3d PUBLIC perceptor log_parser OpenMP::OpenMP_CXX yaml-cpp ${OpenCV_LIBS})
endif()

if(CP_MAPPING3D_NODE_OPTION)
    add_executable(run_mapping_controller run_mapping_controller.cpp)
    target_include_directories(run_mapping_controller PUBLIC ${PATH_LOGREPLAY}/include ${PATH_THIRDPARTY}/perceptor/include ${EIGEN3_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(run_mapping_controller PUBLIC perceptor log_parser OpenMP::OpenMP_CXX yaml-cpp ${OpenCV_LIBS})
endif()

if(CP_WIREDETECT_NODE_OPTION)
    add_executable(run_wire_controller run_wire_controller.cpp)
    target_include_directories(run_wire_controller PUBLIC ${PATH_LOGREPLAY}/include ${PATH_THIRDPARTY}/perceptor/include ${EIGEN3_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(run_wire_controller PUBLIC perceptor log_parser OpenMP::OpenMP_CXX yaml-cpp ${OpenCV_LIBS})
endif()

if(CP_INVASION_NODE_OPTION)
    add_executable(run_invasion_controller run_invasion_controller.cpp)
    message("Include_DIR: ${PATH_LOGREPLAY}") 
    target_include_directories(run_invasion_controller PUBLIC ${PATH_LOGREPLAY}/include ${PATH_THIRDPARTY}/perceptor/include ${EIGEN3_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(run_invasion_controller PUBLIC perceptor log_parser OpenMP::OpenMP_CXX yaml-cpp ${OpenCV_LIBS})
endif()

if(CP_POSTFUSION_NODE_OPTION)
    add_executable(run_postfusion_controller run_postfusion_controller.cpp)
    target_include_directories(run_postfusion_controller PUBLIC ${PATH_LOGREPLAY}/include ${PATH_THIRDPARTY}/perceptor/include ${EIGEN3_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(run_postfusion_controller PUBLIC perceptor log_parser OpenMP::OpenMP_CXX yaml-cpp ${OpenCV_LIBS})
endif()

add_executable(substitude substitude.cpp)
