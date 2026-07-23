# Introduction

This script provides a wrapper for **Cppcheck** static analysis tool

## Prerequisites

- Cppcheck must be installed on your system

## Usage

```
./run_static_analysis_scripts.sh [options]
```

## Options

- `--mode=MODE`: Set the scanning mode
  - `default`: Check entire project in adas_service
  - `dev`: Developer mode for specific checks
  
- `--file=PATH`: Check a specific file (only with dev mode)
- `--folder=PATH`: Check a specific folder (only with dev mode)
- `--output-format=FORMAT`: Set output format
  - `xml` (default)
  - `html`
  
- `--template`: Enable template output format
- `--template-format=TEMPLATE_FORMAT`: Define output format template for dev  mode
  - `gcc`: GCC compatible format
  - `vs`: Visual Studio format
  - `custom`: Define your own template
- `-h, --help`: Display help information

## QuickStart

### Default Analysis
For scanning entire project (adas_service):
```
# Format Options
./run_static_analysis_scripts.sh --ouput-format=<OUTPUT>

# XML output (default)
# 1. Default
./run_static_analysis_scripts.sh

# 2. Explicit command
./run_static_analysis_scripts.sh --output-format=xml

# 3. Short command
./run_static_analysis_scripts.sh -o xml

# HTML output
# 1. Explicit command
./run_static_analysis_scripts.sh --output-format=html

# 2. Short command
./run_static_analysis_scripts.sh -o html
```

### File Analysis
For checking specific files:
```
# Format Options
./run_static_analysis_scripts.sh --mode=<MODE> --file=<*.cpp> --ouput-format=<OUTPUT> --template --template-format=<TEMPLATE_FORMAT>

# XML output (default)
./run_static_analysis_scripts.sh --mode=dev --file=./adas_service/your_file.cpp

# HTML output
# 1. Explicit command
./run_static_analysis_scripts.sh --mode=dev --file=./adas_service/your_file.cpp --output-format=html

# 2. Short command
./run_static_analysis_scripts.sh --mode=dev --file=./adas_service/your_file.cpp -o html

# Terminal output (Do not export XML)
# 1. GCC-style
./run_static_analysis_scripts.sh --mode=dev --file=./adas_service/your_file.cpp --template --template-format=gcc

# Visual Studio style
./run_static_analysis_scripts.sh --mode=dev --file=./adas_service/your_file.cpp --template --template-format=vs
```

### Folder Analysis
For checking specific folders:
```
# Format Options
./run_static_analysis_scripts.sh --mode=<MODE> --folder=<DIRECTORY> --ouput-format=<OUTPUT> --template 

# XML output (default)
./run_static_analysis_scripts.sh --mode=dev --folder=./adas_service/your_folder

# HTML output
./run_static_analysis_scripts.sh --mode=dev --folder=./adas_service/your_folder --output-format=html

# Terminal output
# 1. GCC-style output
./run_static_analysis_scripts.sh --mode=dev --folder=./adas_service/your_folder --template --template-format=gcc

# 2. Visual Studio style
./run_static_analysis_scripts.sh --mode=dev --folder=./adas_service/your_folder --template --template-format=vs
```

## Output

### XML Output (Default)
- Results saved in `cppcheck_report/cppcheck_results.xml`
- For specific files/folders, results are saved with corresponding suffixes

### HTML Output
- Generated when using `--output-format=html`
- Creates an organized report structure:
  ```
  cppcheck_report/
  ├── html_report/
  │   ├── index.html
  │   ├── styles/
  │   │   └── style.css
  │   └── src/
  │       └── [numbered reports].html
  ```
- Opens automatically in default browser if display environment is available

## Default Configuration

The script uses several predefined configurations:

- Project name: "adas_sdk"
- Project directory: "./adas_service/"
- C++ Standard: C++11
- Suppression list: "./script/static_check_scripts/suppressions.txt"
- Excluded directories: test, build, test_ut, test_it

## Notes

- The script automatically organizes and customizes HTML reports
- In dev mode, either `--file` or `--folder` must be specified
- Template formats are only applied when both `--template` and `--template-format` are specified