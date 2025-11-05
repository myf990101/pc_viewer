#! /bin/bash

ECHO_TAG="Cleaner Perceptor"

########################################## support config ##########################################
support_module_options="
    logreplay
    logparser
    logterminal
    bin47totext
"

support_logreplay_options="
    replay
    classify
    mapping3d
    tofmerger
    wiredetect
    invasion
    postfusion
"

# build.sh script compile option list
support_compile_option_list="
    $support_module_options
    $support_logreplay_options
"

# run_logreplay.sh script option list
support_runreplay_option_list="
    $support_logreplay_options
"

########################################## build function ##########################################
# echo success
function echo_success()
{
    if [ $# -ne 1 ]; then
        return
    fi
    # color: green
    echo -e "\033[92m[${ECHO_TAG}] [S] $1 \033[0m"
}

# echo info
function echo_info()
{
    if [ $# -ne 1 ]; then
        return
    fi
    # color: yellow
    echo -e "\033[33m[${ECHO_TAG}] [I] $1 \033[0m"
}

# echo error
function echo_error()
{
    if [ $# -ne 1 ]; then
        return
    fi
    # color: red
    echo -e "\033[31m[${ECHO_TAG}] [E] $1 \033[0m"
}

# build.sh script usage
function usage_build()
{
    echo_info "Usage: ./build.sh [options] "
    echo_info "Common options:"
    echo_info "     --help: -h, display help information"
    echo_info "     --perceptor-path: -p, Specify the perceptor compressed file path and decompress it "
    echo_info "Compile options:"
    echo_info "     --enable-logreplay: build log replay module, default"
    echo_info "     --enable-logparser: build log parse module, default"
    echo_info "     --enable-logterminal: build log terminal module"
    echo_info "     --enable-bin47totext: build bin47 to text module"
    echo_info "     --enable-replay: build logreplay package replay node, default"
    echo_info "     --enable-classify: build logreplay package classify node, default"
    echo_info "     --enable-mapping3d: build logreplay package mapping3d node, default"
    echo_info "     --enable-tofmerger: build logreplay package tofmerger node, default"
    echo_info "     --enable-wiredetect: build logreplay package wiredetect node, default"
    echo_info ""
    echo_info "     --disable-logreplay: do not build log replay module"
    echo_info "     --disable-logparser: do not build log parse module"
    echo_info "     --disable-logterminal: do not build log terminal module, default"
    echo_info "     --disable-bin47totext: do not build bin47 to text module, default"
    echo_info "     --disable-replay: do not build logreplay package replay node"
    echo_info "     --disable-classify: do not build logreplay package classify node"
    echo_info "     --disable-mapping3d: do not build logreplay package mapping3d node"
    echo_info "     --disable-tofmerger: do not build logreplay package tofmerger node,"
    echo_info "     --disable-wiredetect: do not build logreplay package wiredetect node"
    echo_info "For examples:"
    echo_info "     ./build.sh -h"
    echo_info "     ./build.sh -p /home/roborock/perceptor.tar.gz"
    echo_info "     ./build.sh -p /home/roborock/perceptor.tar.gz --enable-logterminal --disable-classify"
    exit 0
}

# check whether parameter 1 exists in the subsquent list
function is_in()
{
    value=$1
    shift
    for var in $*; do
        [ $var == $value ] && return 0
    done
    return 1
}

# enable compile option
function enable_feature()
{
    local feature_name=${1#--enable-} # move prefix --enable-
    if is_in ${feature_name} ${support_compile_option_list}; then
        eval "${feature_name}=1" # create variable ${feature_name}
        return
    fi
    echo_error "compile option $1 is not supported."
    exit 1
}

# disable compile option
function disable_feature()
{
    local feature_name=${1#--disable-} # move prefix --disable-
    if is_in ${feature_name} ${support_compile_option_list}; then
        eval "${feature_name}=0" # create variable ${feature_name}
        return
    fi
    echo_error "compile option $1 is not supported."
    exit 1
}

# init build.sh script compile options
function init_build_options()
{
    # default compile, set variable 1
    logreplay=1
    logparser=1
    logterminal=1
    replay=1
    classify=1
    tofmerger=1
    mapping3d=1
    wiredetection=1
    invasion=1
    postfusion=1

    # set all compile option variable
    for option in ${support_compile_option_list}; do
        if [ ! -n "${!option+x}" ]; then
            # if variable ${option} does not exist, create it
            eval "${option}=0"
        fi
    done

    for option in ${support_compile_option_list}; do
        if [ ! -n "${!option+x}" ]; then
            # check variable ${option} whether exist
            echo_error "${option} is not exist, please check the script."
            exit 1
        fi
    done
}

# extract perceptor include and lib
function extract_perceptor()
{
    perceptor_path=$1
    echo_info "extract perceptor include and lib: ${perceptor_path}."

    if [ ! -f ${perceptor_path} ]; then
        echo_error "extract perceptor failed, please check it: ${perceptor_path}."
        exit 1
    fi

    if [ -d ${current_path}/3rdparty/perceptor ]; then
        rm -r ${current_path}/3rdparty/perceptor
    fi
    mkdir -p ${current_path}/3rdparty/perceptor
    tar -zvxf ${perceptor_path} -C ${current_path}/3rdparty/perceptor
}

# parse build.sh script input parameters
function parse_build_input()
{
    while test "$#" -gt 0; do
        arg="$1"
        shift
        case "${arg}" in
        --help|-h)
            usage_build 
            ;;
        --perceptor-path|-p)
            perceptor_path=$1
            extract_perceptor ${perceptor_path}
            shift
            ;;
        --enable-?*)
            enable_feature ${arg} 
            ;;
        --disable-?*)
            disable_feature ${arg} 
            ;;
        *)
            echo_error "Unknown option: ${arg}"
            usage_build
            ;;
        esac
    done
}

# init cmake build options
function init_cmake_options()
{
    # module options
    if [ ${logreplay} -eq 0 ]; then
        module_compile_options+="-DCP_LOGREPLAY_MODULE_OPTION=OFF "
    else
        module_compile_options+="-DCP_LOGREPLAY_MODULE_OPTION=ON "
    fi

    if [ ${logparser} -eq 0 ]; then
        module_compile_options+="-DCP_LOGPARSER_MODULE_OPTION=OFF "
    else
        module_compile_options+="-DCP_LOGPARSER_MODULE_OPTION=ON "
    fi

    if [ ${logterminal} -eq 0 ]; then
        module_compile_options+="-DCP_LOGTERMINAL_MODULE_OPTION=OFF "
    else
        module_compile_options+="-DCP_LOGTERMINAL_MODULE_OPTION=ON "
    fi

    if [ ${bin47totext} -eq 0 ]; then
        module_compile_options+="-DCP_BIN47TOTEXT_MODULE_OPTION=OFF "
    else
        module_compile_options+="-DCP_BIN47TOTEXT_MODULE_OPTION=ON "
    fi

    # log replay node options
    if [ ${replay} -eq 0 ]; then
        logreplay_node_compile_options+="-DCP_REPLAY_NODE_OPTION=OFF "
    else
        logreplay_node_compile_options+="-DCP_REPLAY_NODE_OPTION=ON "
    fi

    if [ ${classify} -eq 0 ]; then
        logreplay_node_compile_options+="-DCP_CLASSIFY_NODE_OPTION=OFF "
    else
        logreplay_node_compile_options+="-DCP_CLASSIFY_NODE_OPTION=ON "
    fi

    if [ ${mapping3d} -eq 0 ]; then
        logreplay_node_compile_options+="-DCP_MAPPING3D_NODE_OPTION=OFF "
    else
        logreplay_node_compile_options+="-DCP_MAPPING3D_NODE_OPTION=ON "
    fi

    if [ ${tofmerger} -eq 0 ]; then
        logreplay_node_compile_options+="-DCP_TOFMERGER_NODE_OPTION=OFF "
    else
        logreplay_node_compile_options+="-DCP_TOFMERGER_NODE_OPTION=ON "
    fi

    if [ ${wiredetection} -eq 0 ]; then
        logreplay_node_compile_options+="-DCP_WIREDETECT_NODE_OPTION=OFF "
    else
        logreplay_node_compile_options+="-DCP_WIREDETECT_NODE_OPTION=ON "
    fi

    if [ ${invasion} -eq 0 ]; then
        logreplay_node_compile_options+="-DCP_INVASION_NODE_OPTION=OFF "
    else
        logreplay_node_compile_options+="-DCP_INVASION_NODE_OPTION=ON "
    fi

    if [ ${postfusion} -eq 0 ]; then
        logreplay_node_compile_options+="-DCP_POSTFUSION_NODE_OPTION=OFF "
    else
        logreplay_node_compile_options+="-DCP_POSTFUSION_NODE_OPTION=ON "
    fi
} 

# check third party
function check_third_party()
{
    if [ ! -f ${current_path}/3rdparty/perceptor/lib/libperceptor.a ]; then
        echo_error "libperceptor.a not exist, please check it."
        exit 1
    fi
}


########################################## run_logreplay function ##########################################
function usage_logreplay()
{
    echo_info "Usage: ./run_logreplay.sh [options] "
    echo_info "Common options:"
    echo_info "     --help: -h, display help information"
    echo_info "     --input-log: -l, Specify the log replay config: log_directory "
    echo_info "     --start-number: -n, Specify the log replay config: log_start "
    echo_info "     --extract-file: -e, Extract bin47 file from path: log_directory"
    echo_info "Run options:"
    echo_info "     --run-replay: Run log replay module: replay"
    echo_info "     --run-classify: Run log replay module: classify"
    echo_info "     --run-mapping3d: Run log replay module: mapping3d"
    echo_info "     --run-tofmerger: Run log replay module: tof_merger"
    echo_info "     --run-wiredetect: Run log replay module: wire_detection"
    echo_info "For examples:"
    echo_info "     ./run_logreplay.sh -h"
    echo_info "     ./run_logreplay.sh --open-replay"
    echo_info "     ./run_logreplay.sh -e -l /home/roborock/data/ -n 51000 --run-classify"
    exit 0
}

# parse run_logreplay.sh script input parameters
extract=0
log_path=""
map_path=""
wall_path=""
start_number=-1
function parse_logreplay_input()
{
    while test "$#" -gt 0; do
        arg="$1"
        shift
        case "${arg}" in
        --help|-h)
            usage_logreplay 
            ;;
        --input-log|-l)
            log_path=$1
            shift
            ;;
        --start-number|-n)
            start_number=$1
            shift
            ;;
        --map)
            map_path=$1
            shift
            ;;
        --wall)
            wall_path=$1
            shift
            ;;
        --extract-file|-e)
            extract=1
            ;;
        --run-?*)
            run_logreplay ${arg}
            ;;
        *)
            echo_error "Unknown option: ${arg}"
            usage_logreplay
            ;;
        esac
    done
}

# run logplay option
function run_logreplay()
{
    local feature_name=${1#--run-} # move prefix --open-
    if is_in ${feature_name} ${support_runreplay_option_list}; then
        eval "${feature_name}=1" # create variable ${feature_name}
        return
    fi
    echo_error "run logplay option $1 is not supported."
    exit 1
}

# recursive serach log directory for parse log data
function recursive_log_dir
{
    local bin47_tool=$1
    local parse_nav_script=$2
    local cur_dir=$3

    # if cur_dir not a directory, return
    if [ ! -d ${cur_dir} ]; then
        return
    fi

    # use multi process
    {
        cd $cur_dir

        # call bin47totext executable
        `${bin47_tool} >/dev/null 2>&1`

        # call parse_nav_log.py script
        if [ -f "NAV_normal_m.log" ]; then
            `python ${parse_nav} NAV_normal_m.log`
        fi
    }&

    for item in ${cur_dir}/*; do
        if [ ! -d ${item} ]; then
            # if item not a directory, continue
            continue
        fi

        # recursive process
        recursive_log_dir ${bin47_tool} ${parse_nav_script} ${item}
    done
}

# extract log data
function extract_log_data
{
    if [ ! -d $log_path ]; then
        echo_error "log path not exist, please check it: ${log_path}."
        exit 1
    fi
    
    # check Bin47ToText tool
    bin47=${current_path}/tools/bin47_to_text/Bin47ToText
    if [ ! -f ${bin47} ]; then
        echo_error "${bin47} not exist, please check it."
        exit 1
    fi

    if [ ! -x ${bin47} ]; then
        chmod +x ${bin47}
    fi

    # check parse_nav_log.py script
    parse_nav=${current_path}/tools/parse_nav_log.py
    if [ ! -f ${parse_nav} ]; then
        echo_error "${parse_nav} not exist, please check it."
        exit 1
    fi

    # process log data
    recursive_log_dir ${bin47} ${parse_nav} ${log_path}

    # waitting process all
    wait

    # return work path
    cd $current_path

    #
    echo_info "all log data parse over, root path: ${log_path}"
}

# modify yaml config file: log_directory/log_start
function modify_yaml_config()
{
    # log_directory
    if [ ! -z ${log_path} ] && [ -d ${log_path} ]; then # if log_path not empty and log_path is exist
        tmp="log_directory: ${log_path}"
        
        if [ -n "${replay}" ] && [ "${replay}" -eq 1 ]; then # replay.yaml
            if [ -f ./config/replay.yaml ]; then
                # replace log_directory
                row_num=`grep -n "log_directory" ./config/replay.yaml | awk -F':' '{print $1}'`
                sed -i "${row_num}c\\  ${tmp}" ./config/replay.yaml
            fi
        elif [ -n "${classify}" ] && [ "${classify}" -eq 1 ]; then # classify.yaml
            if [ -f ./config/classify.yaml ]; then
                row_num=`grep -n "log_directory" ./config/classify.yaml | awk -F':' '{print $1}'`
                sed -i "${row_num}c\\  ${tmp}" ./config/classify.yaml
            fi
        elif [ -n "${mapping3d}" ] && [ "${mapping3d}" -eq 1 ]; then # mapping3d.yaml
            if [ -f ./config/mapping3d.yaml ]; then
                row_num=`grep -n "log_directory" ./config/mapping3d.yaml | awk -F':' '{print $1}'`
                sed -i "${row_num}c\\  ${tmp}" ./config/mapping3d.yaml
            fi
        elif [ -n "${tofmerger}" ] && [ "${tofmerger}" -eq 1 ]; then # tof_merger.yaml
            if [ -f ./config/tof_merger.yaml ]; then
                row_num=`grep -n "log_directory" ./config/tof_merger.yaml | awk -F':' '{print $1}'`
                sed -i "${row_num}c\\  ${tmp}" ./config/tof_merger.yaml
            fi
        elif [ -n "${wiredetect}" ] && [ "${wiredetect}" -eq 1 ]; then # wire_detection.yaml
            if [ -f ./config/wire_detection.yaml ]; then
                row_num=`grep -n "log_directory" ./config/wire_detection.yaml | awk -F':' '{print $1}'`
                sed -i "${row_num}c\\  ${tmp}" ./config/wire_detection.yaml
            fi
        fi
    fi
    
    # log_start
    if [ "${start_number}" -ge 0 ]; then
        tmp="log_start: ${start_number}"
        
        if [ -n "${replay}" ] && [ "${replay}" -eq 1 ]; then # replay.yaml
            if [ -f ./config/replay.yaml ]; then
                # replace log_start
                row_num=`grep -n "log_start" ./config/replay.yaml | awk -F':' '{print $1}'`
                sed -i "${row_num}c\\  ${tmp}" ./config/replay.yaml
            fi
        elif [ -n "${classify}" ] && [ "${classify}" -eq 1 ]; then # classify.yaml
            if [ -f ./config/classify.yaml ]; then
                row_num=`grep -n "log_start" ./config/classify.yaml | awk -F':' '{print $1}'`
                sed -i "${row_num}c\\  ${tmp}" ./config/classify.yaml
            fi
        elif [ -n "${mapping3d}" ] && [ "${mapping3d}" -eq 1 ]; then # mapping3d.yaml
            if [ -f ./config/mapping3d.yaml ]; then
                row_num=`grep -n "log_start" ./config/mapping3d.yaml | awk -F':' '{print $1}'`
                sed -i "${row_num}c\\  ${tmp}" ./config/mapping3d.yaml
            fi
        elif [ -n "${tofmerger}" ] && [ "${tofmerger}" -eq 1 ]; then # tof_merger.yaml
            if [ -f ./config/tof_merger.yaml ]; then
                row_num=`grep -n "log_start" ./config/tof_merger.yaml | awk -F':' '{print $1}'`
                sed -i "${row_num}c\\  ${tmp}" ./config/tof_merger.yaml
            fi
        elif [ -n "${wiredetect}" ] && [ "${wiredetect}" -eq 1 ]; then # wire_detection.yaml
            if [ -f ./config/wire_detection.yaml ]; then
                row_num=`grep -n "log_start" ./config/wire_detection.yaml | awk -F':' '{print $1}'`
                sed -i "${row_num}c\\  ${tmp}" ./config/wire_detection.yaml
            fi
        fi
    fi

    # mapping3d.yaml
    if [ ! -z ${map_path} ] && [ -d ${map_path} ] && [ -n "${mapping3d}" ] && [ "${mapping3d}" -eq 1 ]; then
        if [ -f ./config/mapping3d.yaml ]; then
            tmp="save_map_directory: ${map_path}"
            row_num=`grep -n "save_map_directory" ./config/mapping3d.yaml | awk -F':' '{print $1}'`
            sed -i "${row_num}c\\  ${tmp}" ./config/mapping3d.yaml
        fi
    fi

    if [ ! -z ${wall_path} ] && [ -f ${wall_path} ] && [ -n "${mapping3d}" ] && [ "${mapping3d}" -eq 1 ]; then
        if [ -f ./config/mapping3d.yaml ]; then
            tmp="map_2d_file: ${wall_path}"
            row_num=`grep -n "map_2d_file" ./config/mapping3d.yaml | awk -F':' '{print $1}'`
            sed -i "${row_num}c\\  ${tmp}" ./config/mapping3d.yaml
        fi
    fi
}