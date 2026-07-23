#!/bin/bash

mode="all"
type="unit-test"

CONFIG_PATH_LOG="./script/test_scripts/logger.sh"

PROJECT_UT_DIRS=()
PROJECT_IT_DIRS=()

# Function to display help
parse_args() {
    echo "Usage: $0 [options]"
    echo ""
    echo "Options:"
    echo "  --mode=MODE            Set the mode of operation (default: $mode)"
    echo "                         Valid options:"
    echo "                           build  - to build the project"
    echo "                           run    - to run the project"
    echo "                           all    - to run entire the project"
    echo ""
    echo "  --type=TYPE            Set the type of tests to execute (default: $type)"
    echo "                         Valid options:"
    echo "                           unit-test          - to run unit tests"
    echo "                           integration-test   - to run integration tests"
    echo ""
    echo "  --target_src=FILE      Specify the test project file to run"
    echo ""
    echo "  --help                 Display this help message"
    exit 0
}

# Function to perform the clean and build process in the given directory
build_test_scripts() {
    local DIR=$1

    LOG_INFO "Navigating to the project directory: $DIR"

    if [ ! -d "$DIR/build" ]; then
        LOG_INFO "Directory does not exist, creating a new one"
        mkdir "$DIR/build" && LOG_INFO "Created build directory"
        cd "$DIR/build" && LOG_INFO "Changed to build directory"
        
        cmake .. 
        if [ $? -ne 0 ]; then
            LOG_ERROR "CMake configuration failed"
            exit 1  # Exit the script with failure status
        fi
        
        make -j4
        if [ $? -ne 0 ]; then
            LOG_ERROR "Build failed during compilation"
            exit 1  # Exit the script with failure status
        fi
        LOG_INFO "Build completed successfully"
    else
        LOG_INFO "Directory exists, removing it"
        rm -rf "$DIR/build" && LOG_INFO "Removed existing build directory"
        mkdir "$DIR/build" && LOG_INFO "Created a new build directory"
        cd "$DIR/build" && LOG_INFO "Changed to build directory"
        
        cmake .. 
        if [ $? -ne 0 ]; then
            LOG_ERROR "CMake configuration failed"
            exit 1  # Exit the script with failure status
        fi
        
        make -j4
        if [ $? -ne 0 ]; then
            LOG_ERROR "Build failed during compilation"
            exit 1  # Exit the script with failure status
        fi
        LOG_INFO "Build completed successfully"

        # # Run Summary of process Test case
        # ctest
        # status=$?
        # if [ $status -ne 0 ]; then
        #     LOG_ERROR "Error: Tests case have failed"
        # else
        #     LOG_INFO "Summary of test results"
        # fi
    fi

    LOG_INFO "Navigating back to the previous directory $( pwd )"
    cd - > /dev/null && LOG_INFO "Changed back to previous directory $( pwd )"
}

run_test_scripts()
{
    local DIR=$1
    local TARGET_SRC=$2

    # Ensure the script exits if any command fails
    set -e

    LOG_INFO "Changing to the build directory of the target source ($DIR/build/$TARGET_SRC)"
    cd "$DIR/build/$TARGET_SRC"

    LOG_INFO "Executing the target source ($TARGET_SRC)"
    ./"$TARGET_SRC"

    LOG_INFO "Navigating back to the previous directory $( pwd )"
    cd - > /dev/null && LOG_INFO "Changed back to previous directory $( pwd )"
}

main()
{
    # Process the arguments
    while [[ "$#" -gt 0 ]]; do
        case $1 in
            --mode=*) mode="${1#*=}"; shift ;;
            --type=*) type="${1#*=}"; shift ;;
            --target_src=*) target_src="${1#*=}"; shift ;;
            --help) parse_args ;;
            *) echo "Unknown parameter passed: $1"; exit 1 ;;
        esac
    done

    source $CONFIG_PATH_LOG

    if [ -n "$target_src" ]; then
        PROJECT_BUILD_UT_DIRS=$(find . -maxdepth 4 -type d -wholename "*$target_src/test_ut")
        PROJECT_BUILD_IT_DIRS=$(find . -maxdepth 4 -type d -wholename "*$target_src/test_it")
        PROJECT_UT_DIRS=$(find . -maxdepth 4 -type d -name "test_ut" -exec sh -c 'find "$1" -maxdepth 1 -mindepth 1 -type d -not -name "build" -name "$2"' _ {} $target_src \;)
        PROJECT_IT_DIRS=$(find . -maxdepth 4 -type d -name "test_it" -exec sh -c 'find "$1" -maxdepth 1 -mindepth 1 -type d -not -name "build" -name "$2"' _ {} $target_src \;)
    else
        PROJECT_BUILD_UT_DIRS=$(find . -maxdepth 4 -type d -name "test_ut")
        PROJECT_BUILD_IT_DIRS=$(find . -maxdepth 4 -type d -name "test_it")
        PROJECT_UT_DIRS=$(find . -maxdepth 4 -type d -name "test_ut" -exec sh -c 'find "$1" -maxdepth 1 -mindepth 1 -type d -not -name "build"' _ {} \;)
        PROJECT_IT_DIRS=$(find . -maxdepth 4 -type d -name "test_it" -exec sh -c 'find "$1" -maxdepth 1 -mindepth 1 -type d -not -name "build"' _ {} \;)
    fi

    ALL_BUILD_DIRS=("${PROJECT_BUILD_UT_DIRS[@]}" "${PROJECT_BUILD_IT_DIRS[@]}")

    ALL_RUN_DIRS=("${PROJECT_UT_DIRS[@]}" "${PROJECT_IT_DIRS[@]}")

    if [[ "$mode" == "build" ]]; then
        if [[ "$type" == "unit-test" ]]; then
            for dir in ${PROJECT_BUILD_UT_DIRS[@]}; do
                build_test_scripts "$dir"
            done
        elif [[ "$type" == "integration-test" ]]; then
            for dir in ${PROJECT_BUILD_IT_DIRS[@]}; do
                build_test_scripts "$dir"
            done
        fi
    fi

    if [[ "$mode" == "run" ]]; then
        if [[ "$type" == "unit-test" ]]; then
            for dir in ${PROJECT_UT_DIRS[@]}; do
                run_test_scripts "$(dirname $dir)" "$(basename $dir)"
            done
        elif [[ "$type" == "integration-test" ]]; then
            for dir in ${PROJECT_IT_DIRS[@]}; do
                run_test_scripts "$(dirname $dir)" "$(basename $dir)"
            done
        fi
    fi
    
    if [[ "$mode" == "all" ]]; then
        for dir in ${ALL_BUILD_DIRS[@]}; do
            build_test_scripts "$dir"
        done
        for dir in ${ALL_RUN_DIRS[@]}; do
            run_test_scripts "$(dirname $dir)" "$(basename $dir)"
        done
    fi

    # Exit the script successfully
    exit 0
}

main "$@"