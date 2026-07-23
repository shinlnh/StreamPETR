#!/bin/bash

local_mode=0
PROJECT_PATH_MODE=""
ORIGIN_PATH=$(pwd)

CONFIG_PATH_ENV="../script/test_scripts/setenv.sh"
CONFIG_PATH_LOG="./script/test_scripts/logger.sh"

PROJECT_UT_DIRS=()
PROJECT_IT_DIRS=()

# Define variable of GCOVR options
readonly ROOT_DIR="./"

readonly REPORT="adas_service"
readonly FILTER_DIR="$REPORT/"

readonly EXCLUDES=".*build/|.*pc_build/|.*test_ut/.*|.*test_it/.*|.*\.h$|.*\.hpp$|.*\.cu$"

readonly OUTPUT_NAME="summary_report"
readonly OUTPUT_REPORT="$REPORT/${OUTPUT_NAME}"

# Define output format
OUTPUT_FORMAT="html"
OUTPUT_OBJ="${OUTPUT_REPORT}/${OUTPUT_NAME}.${OUTPUT_FORMAT}"

GCOVR_OUTPUT_OPTS="--html --html-nested --html-theme github.green"

readonly OTHER_OPTS="--exclude-unreachable-branches --exclude-throw-branches"

# Define project properties
PRJ_NAME="adas_sdk"
BUILD_FOLDER="pc_build"

# Function to display help
parse_args() {
    echo "Usage: $0 [options]"
    echo ""
    echo "Options:"
    echo "  --target_src=DIRECTORY      Specify the test project directory to run"
    echo "  --output_format=FORMAT      Specify output format: xml or html (default is html)"
    echo "  local                       Run in local mode, default is CICD mode"
    echo "  --help                      Display this help message"
    exit 0
}

# clear all duplicate gcno file in build folder
clean_duplicate() {
    # Find the path to adas_sdk
    while [ "$(basename $(pwd))" != "${PRJ_NAME}" ] && [ "$(basename $(pwd))" != "/" ]; do
        cd ..
    done
    # Handle exception
    if [[ "$(pwd)" == "/" ]]; then
        echo "Cannot find the folder adas_sdk"
        exit 1
    else
        echo "Entering the $(pwd)"
    fi

    # Check the existance of pc_build folder
    if [ ! -d ${BUILD_FOLDER} ]; then
        mkdir -p ${BUILD_FOLDER}
    fi

    # Find all file with pattern *.cpp.gnco in adas_service folder 
    # and Delete the same name file in a folder pc_build
    find "${REPORT}" -name "*.cpp.gcno" -type f | while read -r file; do
        find "${BUILD_FOLDER}" -name "$(basename ${file})" -type f -exec rm -f {} \;
    done
}

generate_test_scripts()
{
    local DIR=$1

    cd "$DIR"

    # Define build directory
    BUILD_DIR="pc_build"

    # Create build directory if it doesn't exist
    if [ ! -d "$BUILD_DIR" ]; then
        mkdir -p "$BUILD_DIR"
    fi

    # Navigate to the build directory
    cd "$BUILD_DIR"

    # source ENV
    source $CONFIG_PATH_ENV

    EXEC_STATUS=$?

    # Check if an error occurred
    if [ $EXEC_STATUS -ne 0 ]; then
        echo "Build failed with exit status $EXEC_STATUS"
        exit 1
    else
        echo "Build succeeded"
    fi

    cd ..

    # Clear duplicate files
    clean_duplicate

    # Check if adas_service/summary_report directory exists
    if [ ! -d ${OUTPUT_REPORT} ]; then
        mkdir -p ${OUTPUT_REPORT}
    fi

    LOG_INFO "Starting to generate report..."
    # Generate HTML report by GCOVR 7.2
    gcovr   --root ${ROOT_DIR} --filter ${FILTER_DIR} \
            ${GCOVR_OUTPUT_OPTS} -o ${OUTPUT_OBJ} ${OTHER_OPTS} \
            --exclude ${EXCLUDES} \
            --gcov-ignore-errors=no_working_dir_found > /dev/null 2>&1
    
    if [ -f "${OUTPUT_OBJ}" ]; then
        if [[ $OUTPUT_FORMAT == "html" ]]; then
            LOG_INFO "HTML report generated successfully"

            # Generate custom report for HTML
            LOG_INFO "Generating custom report..."
            custom_html_report ${OUTPUT_REPORT}
            LOG_INFO "Custom report generated successfully"
        else
            LOG_INFO "XML report generated successfully"
        fi
    else
        if [[ $OUTPUT_FORMAT == "html" ]]; then
            LOG_ERROR "Failed to generate HTML report"
        else
            LOG_ERROR "Failed to generate XML report"
        fi
    fi
}

# Reorganize the report by moving .html files to details directory
# and .css files to styles directory
custom_html_report()
{
    local SRC_HTML="src"
    local STYLES_HTML="styles"

    local SRC_HTML_DIR="$1/${SRC_HTML}"
    local STYLES_HTML_DIR="$1/${STYLES_HTML}"
    
    local SUMMARY_REPORT_DIR="$1"
    local SUMMARY_REPORT_NAME=$(basename "$1")
    
    # Create directories
    mkdir -p ${SRC_HTML_DIR}
    mkdir -p ${STYLES_HTML_DIR}

    # Move .html files, but not summary_report.html and summary_report.css
    find ${SUMMARY_REPORT_DIR}/ -maxdepth 1 -type f -name "*.html" ! -name "${SUMMARY_REPORT_NAME}.html" ! -name "${SUMMARY_REPORT_NAME}.css" -exec mv {} ${SRC_HTML_DIR} \;

    # Move .css files
    mv ${SUMMARY_REPORT_DIR}/*.css ${STYLES_HTML_DIR}

    # Replace href attributes in summary_report.html
    if [ -f ${SUMMARY_REPORT_DIR}/${SUMMARY_REPORT_NAME}.html ]; then
        # Replace href to .css file
        sed -i "s/href=\"${SUMMARY_REPORT_NAME}.css\"/href=\"${STYLES_HTML}\/${SUMMARY_REPORT_NAME}.css\"/g" ${SUMMARY_REPORT_DIR}/${SUMMARY_REPORT_NAME}.html

        # Replace href to .html files
        sed -i "s/href=\"\([^\"]*\.html\)\"/href=\"${SRC_HTML}\/\1\"/g" ${SUMMARY_REPORT_DIR}/${SUMMARY_REPORT_NAME}.html
    fi  

    # Replace href attributes in all .html files in src directory
    for file in $(find ${SUMMARY_REPORT_DIR}/${SRC_HTML}/ -type f -name "*.html"); do
        if [ -f $file ]; then
            # Replace href to .css file
            sed -i "s/href=\"${SUMMARY_REPORT_NAME}.css\"/href=\"\.\.\/${STYLES_HTML}\/${SUMMARY_REPORT_NAME}.css\"/g" $file
        fi
    done
}

main()
{
    # Process the arguments
    while [[ "$#" -gt 0 ]]; do
        case $1 in
            --target_src=*) target_src="${1#*=}"; shift ;;
            --output_format=*) OUTPUT_FORMAT="${1#*=}"; shift ;;
            local) local_mode=1; shift ;;
            --help) parse_args ;;
            *) LOG_ERROR "Unknown parameter passed: $1"; exit 1 ;;
        esac
    done

    if [[ $OUTPUT_FORMAT == "xml" ]]; then
        GCOVR_OUTPUT_OPTS="--xml-pretty"
        OUTPUT_OBJ="${OUTPUT_REPORT}/${OUTPUT_NAME}.${OUTPUT_FORMAT}"
    else
        GCOVR_OUTPUT_OPTS="--html --html-nested --html-theme github.green"
        OUTPUT_OBJ="${OUTPUT_REPORT}/${OUTPUT_NAME}.${OUTPUT_FORMAT}"
    fi

    if [[ $local_mode -eq 1 ]]; then
        source $(basename "$CONFIG_PATH_LOG")

        PROJECT_PATH_MODE="../../"

        EXEC_STATUS=$?
        if [ $EXEC_STATUS -eq 1 ]; then
            LOG_WARN "Try to remove argument 'local' to run in CICD mode."
        fi
    else
        source $CONFIG_PATH_LOG

        EXEC_STATUS=$?
        if [ $EXEC_STATUS -eq 1 ]; then
            LOG_WARN "Try to add argument 'local' to run in local mode."
        fi
    fi

    generate_test_scripts ${PROJECT_PATH_MODE}

    # Exit the script successfully
    exit 0
}

main "$@"
