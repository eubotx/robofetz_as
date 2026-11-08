#!/bin/bash
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
source /opt/ros/jazzy/setup.bash
packages=(arena_perception combat_strategizer robot_navigation robot_bringup robot_description robofetz_gazebo)
paths=()
for package in "${packages[@]}"; do paths+=("src/$package"); done
case "${1:-build}" in
    build)
        shift || true
        colcon build --symlink-install --executor sequential --base-paths "${paths[@]}" "$@"
        ;;
    test)
        shift
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
        printf 'Usage: bash .devcontainer/workspace.sh {build|test|smoke|sim} [arguments...]\n' >&2
        exit 2
        ;;
esac
