cmake_minimum_required(VERSION 3.0)
project(cleaner_perceptor)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# system version
file(READ "/etc/os-release" OS_RELEASE)
string(REGEX MATCH "VERSION_ID=\"([0-9]+)\\.([0-9]+)\"" _ ${OS_RELEASE})
math(EXPR UBUNTU_VERSION_NUM "${CMAKE_MATCH_1}*100 + ${CMAKE_MATCH_2}")

if(UBUNTU_VERSION_NUM GREATER 1804)
    message(STATUS "Ubuntu version > 18.04")
else()
    message(STATUS "Ubuntu version <= 18.04")
    add_definitions(-DUBUNTU_VERSION_1804)
endif()

# build options
include(cmake/option.cmake)

# build config
include(cmake/config.cmake)

# define WORK_SPACE_LOCATION
add_definitions(-DWORK_SPACE_LOCATION="${CMAKE_SOURCE_DIR}")

# define ConfigFile
find_file(CONFIG_FILE config.yaml PATHS ${CMAKE_CURRENT_SOURCE_DIR}/config)
add_definitions(-DConfigFile="${CONFIG_FILE}")

# build log replay
if(CP_LOGREPLAY_MODULE_OPTION)
    add_subdirectory(log_replay)
endif()

# build log parse
if(CP_LOGPARSER_MODULE_OPTION)
    add_subdirectory(log_parser)
endif()

# build log terminal
if(CP_LOGTERMINAL_MODULE_OPTION)
    add_subdirectory(log_terminal)
endif()

# build bin47 to text
if(CP_BIN47TOTEXT_MODULE_OPTION)
    add_subdirectory(tools/bin47_to_text)
endif()
