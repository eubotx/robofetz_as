#!/bin/bash
set -eo pipefail

unset WAYLAND_DISPLAY WAYLAND_SOCKET
export XDG_SESSION_TYPE=x11
export DISPLAY=:99
export QT_QPA_PLATFORM=xcb
export QT_X11_NO_MITSHM=1
export LIBGL_ALWAYS_SOFTWARE=1
export XDG_RUNTIME_DIR="$(mktemp -d)"
pids=()
cleanup() {
    trap - EXIT INT TERM
    for pid in "${pids[@]}"; do kill -TERM "$pid" 2>/dev/null || true; done
    wait || true
    rmdir "$XDG_RUNTIME_DIR" 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 0' INT TERM

Xvfb "$DISPLAY" -screen 0 1600x900x24 -nolisten tcp +extension GLX &
pids+=("$!")
for _ in {1..100}; do
    if xdpyinfo >/dev/null 2>&1; then break; fi
    sleep 0.1
done
xdpyinfo >/dev/null

openbox --sm-disable &
pids+=("$!")
x11vnc -display "$DISPLAY" -localhost -rfbport 5900 -forever -shared -nopw &
pids+=("$!")
websockify --web=/usr/share/novnc/ 6080 127.0.0.1:5900 &
pids+=("$!")
ros2 launch robot_bringup robofetz_as.launch.py use_sim:=true "$@" &
pids+=("$!")
wait -n "${pids[@]}"
