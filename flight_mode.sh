#!/usr/bin/env bash
# Switch the C2 boot service between BENCH mode (props off — motor tests) and
# FLIGHT mode (props on — Mission 1/2/waypoints). Props-off is the safe default,
# so enabling flight is always a deliberate act.
#
#   ./flight_mode.sh on      # props ON  -> flight missions allowed
#   ./flight_mode.sh off     # props OFF -> motor tests allowed (default)
#   ./flight_mode.sh status  # what mode is the service in?
set -euo pipefail

SVC="venator-c2.service"
DROPIN_DIR="/etc/systemd/system/${SVC}.d"
DROPIN="${DROPIN_DIR}/props.conf"
SERVER="/home/jetson/Desktop/pixhawk-test/jetson_c2_server.py"
PY="/home/jetson/Desktop/pixhawk-test/venv/bin/python"

case "${1:-status}" in
  on)
    # Props ON is the server default now, so this just clears any bench override.
    echo "==> Switching to FLIGHT mode (props ON)..."
    echo "    Motor tests will be locked out; Mission 1/2/waypoints allowed."
    sudo rm -f "${DROPIN}"
    sudo rmdir "${DROPIN_DIR}" 2>/dev/null || true
    sudo systemctl daemon-reload
    sudo systemctl restart "${SVC}"
    sleep 3
    journalctl -u "${SVC}" -n 3 --no-pager | sed 's/.*python\[[0-9]*\]: //'
    echo "FLIGHT mode active — PROPS MUST BE ON."
    ;;
  off)
    echo "==> Switching to BENCH mode (props OFF)..."
    sudo mkdir -p "${DROPIN_DIR}"
    sudo tee "${DROPIN}" >/dev/null <<EOF
[Service]
ExecStart=
ExecStart=${PY} ${SERVER} --real --props-off
EOF
    sudo systemctl daemon-reload
    sudo systemctl restart "${SVC}"
    sleep 3
    journalctl -u "${SVC}" -n 3 --no-pager | sed 's/.*python\[[0-9]*\]: //'
    echo "BENCH mode active — remove the props before motor tests."
    ;;
  status)
    if [[ -f "${DROPIN}" ]]; then
      echo "Service mode: BENCH (props OFF) — motor tests allowed, flight blocked"
    else
      echo "Service mode: FLIGHT (props ON, default) — flight missions allowed"
    fi
    systemctl is-active "${SVC}"
    ;;
  *)
    echo "Usage: $0 [on|off|status]"; exit 2;;
esac
