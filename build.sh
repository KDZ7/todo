#!/bin/bash

# Variables
BUILD_CMD="colcon build"
VERBOSE_ARGS="--event-handlers console_direct+ --cmake-args -DCMAKE_VERBOSE_MAKEFILE:BOOL=ON"

# Function to handle errors
handle_error() {
    echo -e "\nError: $1\n"
    exit 1
}

# Function to show usage
show_usage() {
    echo -e "\nUsage: $0 [OPTIONS]"
    echo -e "\nOptions:"
    echo -e "\t -b, --build \t\t\t\t Build the workspace"
    echo -e "\t -cb, --clean-build \t\t\t Clean and build the workspace"
    echo -e "\t -s, --symlink \t\t\t\t Use symlinks for install"
    echo -e "\t -v, --verbose \t\t\t\t Enable verbose output"
    echo -e "\t -h, --help \t\t\t\t Show this help message"
    echo -e "\nExample:"
    echo -e "\t $0 -b \t\t\t\t [Build the workspace]"
    echo -e "\t $0 -b [package] \t\t [Build a specific package]"
    echo -e "\t $0 -cb \t\t\t [Clean and build the workspace]"
    echo -e "\t $0 -cb [package] \t\t [Clean and build a specific package]"
    echo -e "\t $0 -b -s \t\t\t [Build the workspace using symlinks]"
    echo -e "\t $0 -b [package] -s \t\t [Build a specific package using symlinks]"
    echo -e "\t $0 -cb -s \t\t\t [Clean and build the workspace using symlinks]"
    echo -e "\t $0 -cb [package] -s \t\t [Clean and build a specific package using symlinks]"
    echo -e "\t $0 -b -v \t\t\t [Build the workspace with verbose output]"
    echo -e "\t $0 -b [package] -v \t\t [Build a specific package with verbose output]"
    echo -e "\t $0 -cb -v \t\t\t [Clean and build the workspace with verbose output]"
    echo -e "\t $0 -cb [package] -v \t\t [Clean and build a specific package with verbose output]"
    echo -e "\t $0 -b -s -v \t\t\t [Build the workspace using symlinks with verbose output]"
    echo -e "\t $0 -b [package] -s -v \t\t [Build a specific package using symlinks with verbose output]"
    echo -e "\t $0 -cb -s -v \t\t\t [Clean and build the workspace using symlinks with verbose output]"
    echo -e "\t $0 -cb [package] -s -v \t [Clean and build a specific package using symlinks with verbose output]\n"
    exit "$1"
}

clean_workspace() {
    echo -e "\nCleaning the workspace...\n"
    echo -e "\nRemoving build and install directories...\n"
    rm -rf build install log
    if [ $? -ne 0 ]; then
        handle_error "Failed to remove 'build', 'install', or 'log' directories from the root directory."
    fi
    echo -e "\nRemoving other files specific extensions...\n"
    rm -rf *.pdf *.gv *.out
    echo -e "\nWorkspace has been successfully cleaned.\n"
}

build_workspace() {
    local package="$1"
    local args="$2"
    
    echo -e "\nBuilding the workspace...\n"
    if [ -n "$package" ]; then
        echo -e "Building package: $package\n"
        $BUILD_CMD --packages-select "$package" $args
    else
        $BUILD_CMD $args
    fi
    
    if [ $? -ne 0 ]; then
        handle_error "Failed to build the workspace."
    fi
    echo -e "\nWorkspace build completed successfully.\n"
}

exec() {
    local action=""
    local args=""
    local package=""

    while [[ $# -gt 0 ]]; do
        case "$1" in
            -h|--help)
                show_usage 0
                ;;
            -b|--build)
                action="build"
                shift
                if [[ $1 != -* ]] && [ -n "$1" ]; then
                    package="$1"
                    shift
                fi
                ;;
            -cb|--clean-build)
                action="clean-build"
                shift
                if [[ $1 != -* ]] && [ -n "$1" ]; then
                    package="$1"
                    shift
                fi
                ;;
            -s|--symlink)
                args+=" --symlink-install"
                shift
                ;;
            -v|--verbose)
                args+=" $VERBOSE_ARGS"
                shift
                ;;
            *)
                if [ -z "$package" ] && [[ $1 != -* ]]; then
                    package="$1"
                    shift
                else
                    echo "Unknown option: $1"
                    show_usage 1
                fi
                ;;
        esac
    done

    # check if action is specified
    if [ -z "$action" ]; then
        echo -e "Error: No action specified (-b or -cb required)"
        show_usage 1
    fi

    # Execute the specified action
    case "$action" in
        "build")
            build_workspace "$package" "$args"
            ;;
        "clean-build")
            clean_workspace
            build_workspace "$package" "$args"
            ;;
        *)
            handle_error "Invalid action: $action"
            ;;
    esac
}

exec "$@"