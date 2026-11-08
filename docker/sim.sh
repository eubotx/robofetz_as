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
declare -A process_names=()

start_process() {
    local name=$1
    shift
    "$@" &
    pids+=("$!")
    process_names[$!]=$name
}

report_exit() {
    printf '%s exited (status %d).\n' "${process_names[$1]}" "$2" >&2
    exit "$2"
}

check_processes() {
    local pid status
    for pid in "${pids[@]}"; do
        if ! kill -0 "$pid" 2>/dev/null; then
            status=0
            wait "$pid" || status=$?
            report_exit "$pid" "$status"
        fi
    done
}

wait_for() {
    local name=$1
    shift
    for _ in {1..100}; do
        check_processes
        if "$@" >/dev/null 2>&1; then
            return
        fi
        sleep 0.1
    done
    check_processes
    printf 'Timed out waiting for %s.\n' "$name" >&2
    exit 1
}

cleanup() {
    trap - EXIT INT TERM
    for pid in "${pids[@]}"; do kill -TERM "$pid" 2>/dev/null || true; done
    wait || true
    rmdir "$XDG_RUNTIME_DIR" 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 0' INT TERM

start_process Xvfb Xvfb "$DISPLAY" -screen 0 1600x900x24 -nolisten tcp +extension GLX
wait_for "Xvfb display $DISPLAY" xdpyinfo

start_process Openbox openbox --sm-disable
start_process x11vnc x11vnc -display "$DISPLAY" -localhost -rfbport 5900 -forever -shared -nopw
start_process websockify websockify --web=/usr/share/novnc/ 6080 127.0.0.1:5900
wait_for 'the noVNC page and VNC server' python3 -c '
import http.client
import socket

connection = http.client.HTTPConnection("127.0.0.1", 6080, timeout=0.5)
connection.request("GET", "/vnc.html")
if connection.getresponse().status != 200:
    raise SystemExit(1)
connection.close()
with socket.create_connection(("127.0.0.1", 5900), timeout=0.5) as vnc:
    raise SystemExit(vnc.makefile("rb").read(4) != b"RFB ")
'
check_processes
printf 'Simulation desktop ready on container port 6080. Open /vnc.html?autoconnect=1&resize=scale at its host address.\n'

start_process 'ROS launch' ros2 launch robot_bringup robofetz_as.launch.py use_sim:=true "$@"
check_processes
status=0
wait -n -p exited_pid "${pids[@]}" || status=$?
if [[ -z "$exited_pid" ]]; then
    check_processes
    printf 'Could not wait for the simulation processes (status %d).\n' "$status" >&2
    exit 1
fi
report_exit "$exited_pid" "$status"
