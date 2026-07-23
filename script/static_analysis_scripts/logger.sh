#!/bin/bash

# Define color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
BLUE='\033[0;34m'
MAGENTA='\033[0;35m'
CYAN='\033[0;36m'
RESET='\033[0m' # Reset Color

# Function to print log messages
LOG_ERROR() {
    echo -e "${RED}[ERROR]: $1${RESET}"
    exit 1
}

LOG_WARN() {
    echo -e "${YELLOW}[WARN]: $1${RESET}"
}

LOG_INFO() {
    echo -e "${GREEN}[INFO]: $1${RESET}"
}

LOG_DEBUG() {
    echo -e "${BLUE}[DEBUG]: $1${RESET}"
}