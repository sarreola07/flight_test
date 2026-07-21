#!/usr/bin/env bash
# AI camera tracking — on-demand toggle.
#
# Starts / stops the OAK-D person tracker (camera_publisher.py) as a background
# USER process. It is fully decoupled from the drone's MAVLink link and from any
# systemd service: it only reads the USB camera and publishes person coordinates
# on UDP 127.0.0.1:5005. Starting or stopping it NEVER opens /dev/ttyACM0, so it
# cannot restart the drone or interrupt background telemetry.
#
# Usage:
#   ./ai_camera.sh            # toggle: start if stopped, stop if running
#   ./ai_camera.sh start|stop|restart|status
set -uo pipefail

REPO="$(cd "$(dirname "$0")" && pwd)"
CAMERA_PY="${CAMERA_PY:-/home/jetson/oak_drone_project/depthai-env/bin/python}"
SCRIPT="${REPO}/camera_publisher.py"

STATE_DIR="${XDG_STATE_HOME:-$HOME/.local/state}/ai-camera"
PIDFILE="${STATE_DIR}/camera.pid"
LOGFILE="${STATE_DIR}/camera.log"
mkdir -p "${STATE_DIR}"

notify() {
    # Desktop popup when launched from the GNOME session; always print too.
    if command -v notify-send >/dev/null 2>&1; then
        notify-send -a "AI Camera" -i "${REPO}/assets/ai-camera.png" "AI Camera" "$1" 2>/dev/null || true
    fi
    echo "AI Camera: $1"
}

# Echo the live camera_publisher PID, or nothing.
running_pid() {
    local pid=""
    if [[ -f "${PIDFILE}" ]]; then
        pid="$(cat "${PIDFILE}" 2>/dev/null || true)"
        if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null \
           && grep -qa "camera_publisher.py" "/proc/${pid}/cmdline" 2>/dev/null; then
            echo "${pid}"
            return 0
        fi
    fi
    # Fallback: a camera started outside this pidfile. Match the exact script
    # path (won't appear in incidental command lines) and exclude ourselves.
    pgrep -f "${SCRIPT}" 2>/dev/null | grep -vx "$$" | head -n1
}

start() {
    local pid; pid="$(running_pid)"
    if [[ -n "${pid}" ]]; then
        notify "already running (PID ${pid})."
        return 0
    fi
    if [[ ! -x "${CAMERA_PY}" ]]; then
        notify "DepthAI python not found at ${CAMERA_PY}."
        return 1
    fi
    echo "=== $(date) starting camera_publisher ===" >> "${LOGFILE}"
    nohup "${CAMERA_PY}" "${SCRIPT}" >> "${LOGFILE}" 2>&1 &
    local newpid=$!
    echo "${newpid}" > "${PIDFILE}"
    sleep 2
    if kill -0 "${newpid}" 2>/dev/null; then
        notify "started (PID ${newpid}) — tracking on UDP 5005."
    else
        notify "failed to start — see ${LOGFILE}"
        tail -n 5 "${LOGFILE}" 2>/dev/null || true
        rm -f "${PIDFILE}"
        return 1
    fi
}

stop() {
    local pid; pid="$(running_pid)"
    if [[ -z "${pid}" ]]; then
        notify "not running."
        rm -f "${PIDFILE}"
        return 0
    fi
    kill "${pid}" 2>/dev/null || true
    for _ in $(seq 1 10); do
        kill -0 "${pid}" 2>/dev/null || break
        sleep 0.3
    done
    kill -0 "${pid}" 2>/dev/null && kill -9 "${pid}" 2>/dev/null || true
    rm -f "${PIDFILE}"
    notify "stopped (was PID ${pid})."
}

status() {
    local pid; pid="$(running_pid)"
    if [[ -n "${pid}" ]]; then
        echo "AI camera: RUNNING (PID ${pid})"
    else
        echo "AI camera: stopped"
    fi
}

case "${1:-toggle}" in
    start)   start ;;
    stop)    stop ;;
    restart) stop; sleep 1; start ;;
    status)  status ;;
    toggle)  if [[ -n "$(running_pid)" ]]; then stop; else start; fi ;;
    *) echo "Usage: $0 [start|stop|restart|status|toggle]"; exit 2 ;;
esac
