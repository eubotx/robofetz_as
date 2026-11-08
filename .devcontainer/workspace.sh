#!/bin/bash
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
source /opt/ros/jazzy/setup.bash
packages=(arena_perception combat_strategizer robot_navigation robot_bringup robot_description robofetz_gazebo)
paths=()
for package in "${packages[@]}"; do paths+=("src/$package"); done

lock_workspace() {
    mkdir -p log
    exec 9>log/.workspace.lock
    if ! flock -n 9; then
        printf 'Another workspace build or test is running. Try again when it finishes.\n' >&2
        exit 1
    fi
}

case "${1:-build}" in
    build|clean-build)
        action=${1:-build}
        shift || true
        lock_workspace
        if [[ "$action" == clean-build ]]; then
            if (( $# != 0 )); then
                printf 'clean-build takes no arguments and rebuilds the full simulation workspace.\n' >&2
                exit 2
            fi
            if [[ -L build || -L install ]]; then
                printf 'Refusing to clean symlinked build or install directories.\n' >&2
                exit 1
            fi
            printf 'Clearing generated build and install outputs. Keeping log/.\n'
            mkdir -p build install
            find build install -xdev -mindepth 1 -delete
        fi
        colcon build --symlink-install --executor sequential --base-paths "${paths[@]}" "$@"
        ;;
    test)
        shift
        lock_workspace
        source install/local_setup.bash
        status=0
        colcon test --base-paths "${paths[@]}" --packages-select "${packages[@]}" --return-code-on-test-failure "$@" || status=$?
        colcon test-result --verbose || status=$?
        exit "$status"
        ;;
    smoke)
        exec ros-env robofetz-smoke
        ;;
    sim)
        shift
        exec ros-env bash docker/sim.sh "$@"
        ;;
    *)
        printf 'Usage: bash .devcontainer/workspace.sh {build|clean-build|test|smoke|sim} [arguments...]\n' >&2
        exit 2
        ;;
esac
