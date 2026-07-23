#!/bin/bash

# Project Configuration
PRJ_NAME="adas_sdk"

# Configure Path of Scanning
PROJECT_DIR="./adas_service/"
REPORT_DIR="cppcheck_report"
XML_REPORT="${REPORT_DIR}/cppcheck_results.xml"
HTML_REPORT_DIR="${REPORT_DIR}/html_report"

# Optional Configuration Cppcheck
ENALBLE_LISTS="all"
XML_CONFIG="--xml --xml-version=2"
ERROR_EXITCODE=1
SUPPRESSION_LISTS="./script/static_analysis_scripts/suppressions.txt"
PLATFORM_TYPE="native"
VERSION_CPP="--std=c++11"
EXCLUDES_CONFIG="  -i test \
            -i build \
            -i test_ut \
            -i test_it"
OTHERS_OPTS="--check-level=exhaustive"

# Parse Arguments Configuration
MODE="default"
OUTPUT_FORMAT="xml"
ENABLE_TEMPLATE=false
TEMPLATE_TYPE=""

# Function to display help
parse_args() {
    echo "Usage: $0 [options]"
    echo ""
    echo "Options:"
    echo "  --mode=MODE             Set the mode of scanning static check:"
    echo "                          default     - Check entire project in adas_service."
    echo "                          dev         - Developer mode with specific checks."
    echo "  --file=PATH             Check specific file     (with dev mode)."
    echo "  --folder=PATH           Check specific folder   (with dev mode)."
    echo "  --output-format=FORMAT  Set output format (html or xml)."
    echo "                          Default is xml."
    echo "  --template          Enable template output format"
    echo "  --template-format=TEMPLATE_FORMAT       Define output format template for dev mode:"
    echo "                          gcc         - GCC compatible format"
    echo "                          vs         - Visual Studio format"
    echo "                          custom     - Define your own template"
    echo "  -h, --help              Print this help."
    exit 0
}

setup_env()
{
    while [ "$(basename $(pwd))" != "${PRJ_NAME}" ] && [ "$(basename $(pwd))" != "/" ]; do
        cd ..
        
    done

    # Source Environment
    source ./script/static_analysis_scripts/logger.sh
}

# Check file existence
validate_file() {
    local file_path=$1
    if [ ! -f "$file_path" ]; then
        LOG_ERROR "File not found: $file_path"
        exit 1
    fi
}

# Check folder existence
validate_folder() {
    local folder_path=$1
    if [ ! -d "$folder_path" ]; then
        LOG_ERROR "Directory not found: $folder_path"
        exit 1
    fi
}

get_template_format() {
    local template_type=$1
    case $template_type in
        "gcc")
            echo "{file}:{line}: {severity}: {message}"
            ;;
        "vs")
            echo "{file}({line}): {severity}: {message}"
            ;;
        "custom="*)
            echo "${template_type#custom=}"
            ;;
        *)
            echo "{file}:{line}:{severity}:{message}"  # default
            ;;
    esac
}

# Run Cppcheck for single file
run_cppcheck_file() {
    local file_path=$1
    local report_suffix=$(basename "$file_path" .cpp)
    
    LOG_INFO "Checking file: $file_path"
    
    XML_REPORT="${REPORT_DIR}/cppcheck_${report_suffix}.xml"
    HTML_REPORT_DIR="${REPORT_DIR}/html_report_${report_suffix}"
    
    if [ -n "$TEMPLATE_TYPE" ]; then
        # Run with template format
        local template_format=$(get_template_format "$TEMPLATE_TYPE")
        cppcheck    --enable=${ENALBLE_LISTS} \
                    --suppressions-list=${SUPPRESSION_LISTS} \
                    --error-exitcode=${ERROR_EXITCODE} \
                    --platform=${PLATFORM_TYPE} \
                    ${VERSION_CPP} \
                    --template="${template_format}" \
                    ${OTHERS_OPTS} \
                    "$file_path" 2>&1
    else
        # Run with XML format
        cppcheck    --enable=${ENALBLE_LISTS} \
                    ${XML_CONFIG} \
                    --suppressions-list=${SUPPRESSION_LISTS} \
                    --error-exitcode=${ERROR_EXITCODE} \
                    --platform=${PLATFORM_TYPE} \
                    ${VERSION_CPP} \
                    ${OTHERS_OPTS} \
                    "$file_path" 2> "$XML_REPORT"
    fi
    return $?
}

# Run Cppcheck for specific folder
run_cppcheck_folder() {
    local folder_path=$1
    local report_suffix=$(basename "$folder_path")
    
    LOG_INFO "Checking folder: $folder_path"
    
    XML_REPORT="${REPORT_DIR}/cppcheck_${report_suffix}.xml"
    HTML_REPORT_DIR="${REPORT_DIR}/html_report_${report_suffix}"
    
    if [ -n "$TEMPLATE_TYPE" ]; then
        # Run with template format
        local template_format=$(get_template_format "$TEMPLATE_TYPE")
        cppcheck    --enable=${ENALBLE_LISTS} \
                    --suppressions-list=${SUPPRESSION_LISTS} \
                    --error-exitcode=${ERROR_EXITCODE} \
                    ${EXCLUDES_CONFIG} \
                    --platform=${PLATFORM_TYPE} \
                    ${VERSION_CPP} \
                    ${template_format} \
                    ${OTHERS_OPTS} \
                    "$folder_path" 2>&1
    else
        # Run with XML format
        cppcheck    --enable=${ENALBLE_LISTS} \
                    ${XML_CONFIG} \
                    --suppressions-list=${SUPPRESSION_LISTS} \
                    --error-exitcode=${ERROR_EXITCODE} \
                    ${EXCLUDES_CONFIG} \
                    --platform=${PLATFORM_TYPE} \
                    ${VERSION_CPP} \
                    ${OTHERS_OPTS} \
                    "$folder_path" 2> "$XML_REPORT"
    fi
    return $?
}

# Run Cppcheck analysis for adas_service
run_cppcheck() {
    echo -e "${YELLOW}Starting Cppcheck analysis...${RESET}"
    echo "Analyzing project in: $PROJECT_DIR"
    
    # Run Cppcheck with detailed options
    cppcheck    --enable=${ENALBLE_LISTS} \
                ${XML_CONFIG} \
                --suppressions-list=${SUPPRESSION_LISTS} \
                --error-exitcode=${ERROR_EXITCODE} \
                ${EXCLUDES_CONFIG} \
                --platform=${PLATFORM_TYPE} \
                ${VERSION_CPP} \
                ${OTHERS_OPTS} \
                "$PROJECT_DIR" 2> "$XML_REPORT"
    return $?
}

reorganize_html_report() {
    local report_dir=$1
    
    # Create directories
    mkdir -p "${report_dir}/src"
    mkdir -p "${report_dir}/styles"

    # Move style.css to styles dir
    mv "${report_dir}/style.css" "${report_dir}/styles/"

    # Move all numbered html files to src dir 
    for file in "${report_dir}"/*.html; do
        if [[ $file != *"index.html" ]]; then
            mv "$file" "${report_dir}/src/"
        fi
    done

    # Update index.html references
    sed -i 's/href="\([0-9]\+\.html\)/href="src\/\1/g' "${report_dir}/index.html"
    sed -i 's/href="style\.css/href="styles\/style.css/g' "${report_dir}/index.html"
}

customize_html_report() {
    local report_dir=$1

    LOG_INFO "Customizing HTML report..."
    # reorganize files .html
    reorganize_html_report "$report_dir"
    # update links in .html
    update_src_links "$report_dir"
    LOG_INFO "HTML report customized successfully"
}
# Update all links in the source html files
update_src_links() {
    local report_dir=$1
    
    # Update links in source files
    for file in "${report_dir}"/src/*.html; do
        sed -i 's/href="\([0-9]\+\.html\)/href="..\/src\/\1/g' "$file"
        sed -i 's/href="style\.css/href="..\/styles\/style.css/g' "$file"
        sed -i 's/href="index\.html/href="..\/index.html/g' "$file"
    done
}

# Generate HTML report
generate_html_report() {
    echo -e "${YELLOW}Generating HTML report...${RESET}"
    
    cppcheck-htmlreport \
        --file="$XML_REPORT" \
        --report-dir="$HTML_REPORT_DIR" \
        --source-dir=. \
        --title="Cppcheck Analysis Report"

    customize_html_report "$HTML_REPORT_DIR"

    if [ $? -eq 0 ]; then
        echo -e "${GREEN}HTML report generated successfully${RESET}"
        
        # Display information of reports
        if [ -n "$DISPLAY" ]; then
            if command -v xdg-open &> /dev/null; then
                xdg-open "$HTML_REPORT_DIR/index.html"
            elif command -v open &> /dev/null; then
                open "$HTML_REPORT_DIR/index.html"
            fi
        fi
    else
        echo -e "${RED}Failed to generate HTML report${RESET}"
    fi
}

main() {
    setup_env

    # Create cppcheck_report directory
    if [ ! -d "$REPORT_DIR" ]; then
        mkdir -p "$REPORT_DIR"
    fi

    # Process the arguments
    while [[ "$#" -gt 0 ]]; do
        case $1 in
            --mode=*|-m)
                if [[ $1 == --mode=* ]]; then
                    MODE="${1#*=}"
                else
                    MODE="$2"
                    shift
                fi
                shift
                ;;
            --file=*|-f) 
                if [[ $1 == --file=* ]]; then
                    file_path="${1#*=}"
                else
                    file_path="$2"
                    shift
                fi
                shift
                ;;
            --folder=*|-d) 
                if [[ $1 == --folder=* ]]; then
                    folder_path="${1#*=}"
                else
                    folder_path="$2"
                    shift
                fi
                shift
                ;;
            --output-format=*|-o)
                if [[ $1 == --output-format=* ]]; then
                    OUTPUT_FORMAT="${1#*=}"
                else
                    OUTPUT_FORMAT="$2"
                    shift
                fi
                if [[ "$OUTPUT_FORMAT" != "html" && "$OUTPUT_FORMAT" != "xml" ]]; then
                    LOG_ERROR "Invalid output format. Use 'html' or 'xml'"
                    exit 1
                fi
                shift
                ;;
            --template)
                ENABLE_TEMPLATE=true
                shift
                ;;
            --template-format=*)
                TEMPLATE_TYPE="${1#*=}"
                shift
                ;;
            -h|--help) parse_args ;;
            *) LOG_ERROR "Unrecognized command line option: $1"; exit 1 ;;
        esac
    done

    # Validate Template format
    if [ "$ENABLE_TEMPLATE" = true ] && [ -z "$TEMPLATE_TYPE" ]; then
        LOG_ERROR "Template format must be specified when using --template"
        exit 1
    fi

    if [ "$ENABLE_TEMPLATE" = false ] && [ -n "$TEMPLATE_TYPE" ]; then
        LOG_INFO "Template will be ignored without --template flag"
        TEMPLATE_TYPE=""
    fi

    LOG_INFO "=== Starting Static Code Analysis with mode: $MODE ==="
    
    case $MODE in
        "dev")
            if [ -n "$file_path" ]; then
                validate_file "$file_path"
                run_cppcheck_file "$file_path"
            elif [ -n "$folder_path" ]; then
                validate_folder "$folder_path"
                run_cppcheck_folder "$folder_path"
            else
                LOG_ERROR "Dev mode requires --file or --folder parameter"
                parse_args
            fi
            ;;
        "default")
            run_cppcheck
            ;;
    esac

    if [ "$OUTPUT_FORMAT" = "html" ]; then
        generate_html_report
    else
        LOG_INFO "Open XML report: $XML_REPORT"
    fi
}

main "$@"