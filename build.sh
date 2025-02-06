#!/bin/bash

# Variables
BUILD_CMD="colcon build"
VERBOSE_ARGS="--event-handlers console_direct+ --cmake-args -DCMAKE_VERBOSE_MAKEFILE:BOOL=ON"
ARGS=""

# Function to handle errors
handle_error() {
    echo -e "\nError: $1\n"
    exit 1
}

# Function to show usage
show_usage() {
    echo -e "\nUsage: $0 [OPTIONS]\n"
    echo -e "Options:"
    echo -e "\t -s, --symlink \t\t Use symlinks for install"
    echo -e "\t -v, --verbose \t\t Enable verbose output"
    echo -e "\t -h, --help \t\t Show this help message\n"
    echo -e "Example:"
    echo -e "\t $0 --symlink --verbose\n"  
    exit "$1"
}

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case "$1" in
        -h|--help)
            show_usage 0
            ;;
        -s|--symlink)
            ARGS+=" --symlink-install"
            shift
            ;;
        -v|--verbose)
            ARGS+=" $VERBOSE_ARGS"
            shift
            ;;
        *)
            show_usage 1
            ;;
    esac
done

# Build the workspace
$BUILD_CMD $ARGS

if [ $? -ne 0 ]; then
    handle_error "Failed to build the workspace."
fi

echo -e "\nWorkspace build completed successfully.\n"