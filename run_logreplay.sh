#! /bin/bash

source ./script/helper.sh
current_path=$(cd `dirname $0`;pwd)

# parse log replay input parameters
parse_logreplay_input "$@"

# check build result
if [ ! -f devel/setup.sh ]; then
    echo_error "The devel/setup.sh file does not exist. Please compile and run it again."
    exit 1
fi
source devel/setup.sh

# extract log data: bin47
if [ ${extract} -eq 1 ]; then
    extract_log_data
fi

# replace yaml config file log_directory/log_start
modify_yaml_config

# run log replay
# replay
if [ -n "${replay}" ] && [ "${replay}" -eq 1 ]; then
    if [ -f ./devel/lib/log_replay/replay ]; then
        roslaunch log_replay log_replay.launch
        exit 0
    else
        echo_error "the executable file ./devel/lib/log_replay/replay not exist, please check it."
        exit 1
    fi
fi

# classify
if [ -n "${classify}" ] && [ "${classify}" -eq 1 ]; then
    if [ -f ./devel/lib/log_replay/classify ]; then
        roslaunch log_replay classify.launch
        exit 0
    else
        echo_error "the executable file ./devel/lib/log_replay/classify not exist, please check it."
        exit 1
    fi
fi

# mapping3d
if [ -n "${mapping3d}" ] && [ "${mapping3d}" -eq 1 ]; then
    if [ -f ./devel/lib/log_replay/mapping3d ]; then
        roslaunch log_replay mapping3d.launch
        exit 0
    else
        echo_error "the executable file ./devel/lib/log_replay/mapping3d not exist, please check it."
        exit 1
    fi
fi

# tof_merger
if [ -n "${tofmerger}" ] && [ "${tofmerger}" -eq 1 ]; then
    if [ -f ./devel/lib/log_replay/tof_merger ]; then
        roslaunch log_replay tof_merger.launch
        exit 0
    else
        echo_error "the executable file ./devel/lib/log_replay/tof_merger not exist, please check it."
        exit 1
    fi
fi

# wire_detection
if [ -n "${wiredetect}" ] && [ "${wiredetect}" -eq 1 ]; then
    if [ -f ./devel/lib/log_replay/wire_detection ]; then
        roslaunch log_replay wire_detection.launch
        exit 0
    else
        echo_error "the executable file ./devel/lib/log_replay/wire_detection not exist, please check it."
        exit 1
    fi
fi