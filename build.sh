#! /bin/bash

source ./script/helper.sh
current_path=$(cd `dirname $0`;pwd)

# init all build options
init_build_options

# parse input parameters
parse_build_input "$@"

# init compile options
module_compile_options=""
logreplay_node_compile_options=""
init_cmake_options

# check third party
check_third_party

# print compile info
compile_date=$(date)
compile_person=$(git config user.name)
compile_branch=$(git rev-parse --abbrev-ref HEAD)
commit_date=$(git log --pretty=format:“%cd” HEAD -1)
commit_hash=$(git rev-parse HEAD)
compile_info=(
    "compile date: ${compile_date}"
    "compile person: ${compile_person}"
    "compile branch: ${compile_branch}"
    "commit date: ${commit_date}"
    "commit hash: ${commit_hash}"
    "module compile options: ${module_compile_options}"
    "log replay node compile options: ${logreplay_node_compile_options}"
    "devel path: ${current_path}/devel"
    "build path: ${current_path}/build"
)
for val in "${compile_info[@]}"; do
    echo_info "${val}"
done

# cmake build and install
if [ -d ${current_path}/build ]; then
    rm -r ${current_path}/build
fi
if [ -d ${current_path}/devel ]; then
    rm -r ${current_path}/devel
fi
mkdir ${current_path}/build && cd ${current_path}/build

cmake \
    -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    -DCATKIN_DEVEL_PREFIX=$current_path/devel \
    ${module_compile_options} \
    ${logreplay_node_compile_options} \
    ..
if [ $? -ne 0 ]; then
    echo_error "cmake failed, please check cmake code, ret $?."
    exit 1
fi

make -j8
if [ $? -ne 0 ]; then
    echo_error "make failed, please check source code, ret $?."
    exit 1
fi

echo_success "all target build over."
