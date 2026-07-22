#!/usr/bin/env bash
# Install the Venator C2 server as a boot service — the Jetson then starts the
# LoRa command link automatically on power-on (zero-touch). Asks for sudo once.
#
#   bash install_boot_service.sh
#
# The service runs the server with --real (connects the Pixhawk) and props OFF,
# so remotely only motor tests / camera run; flight stays gated. Nothing arms on
# boot. Stop it (below) when you want to use missions.py interactively, since
# both want the Pixhawk serial port.
set -euo pipefail

REPO="$(cd "$(dirname "$0")" && pwd)"
SVC="venator-c2.service"

[[ -x "${REPO}/venv/bin/python" ]] || { echo "Missing venv — run: bash setup.sh"; exit 1; }

echo "==> Installing ${SVC} ..."
sudo install -m 644 "${REPO}/systemd/${SVC}" "/etc/systemd/system/${SVC}"
sudo systemctl daemon-reload
sudo systemctl enable "${SVC}"
sudo systemctl restart "${SVC}"
sleep 2
echo
sudo systemctl --no-pager --lines=8 status "${SVC}" || true

echo
echo "Done — the C2 server starts on every boot."
echo "  live logs : journalctl -u ${SVC} -f"
echo "  stop      : sudo systemctl stop ${SVC}      (frees the Pixhawk for missions.py)"
echo "  start     : sudo systemctl start ${SVC}"
echo "  disable   : sudo systemctl disable ${SVC}"
