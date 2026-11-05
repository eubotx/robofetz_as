#!/bin/bash
set -eo pipefail
for directory in /ws/build /ws/install /ws/log; do
    mkdir -p "$directory"
    if [ "$(stat -c '%u:%g' "$directory")" != "$(id -u robot):$(id -g robot)" ]; then
        chown -R robot:robot "$directory"
    fi
done
exec gosu robot ros-env "$@"
