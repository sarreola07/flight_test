#!/usr/bin/env bash
# Install the hexacopter environment:
#   1. Desktop launcher icon (runs the mission menu)
#   2. oak-camera boot service (starts the OAK-D camera publisher on power-on)
#
# Run from your desktop terminal:  bash install.sh
# Asks for your sudo password once (to install the system service).
set -euo pipefail

REPO="$(cd "$(dirname "$0")" && pwd)"
SERVICE="oak-camera.service"
CAMERA_PY="/home/jetson/oak_drone_project/depthai-env/bin/python"

echo "==> Repo: ${REPO}"

# --- sanity checks -----------------------------------------------------------
[[ -x "${REPO}/venv/bin/python" ]] || { echo "Missing venv — run: bash setup.sh"; exit 1; }
[[ -x "${CAMERA_PY}" ]] || echo "WARNING: DepthAI env not found at ${CAMERA_PY} (camera service will fail until it exists)."

# --- 1. desktop launcher -----------------------------------------------------
echo "==> Installing desktop launcher..."
chmod +x "${REPO}/run_missions.sh"
DESK="${HOME}/Desktop/hexacopter-mission.desktop"
APPS="${HOME}/.local/share/applications"
mkdir -p "${APPS}"
install -m 755 "${REPO}/hexacopter-mission.desktop" "${DESK}"
install -m 644 "${REPO}/hexacopter-mission.desktop" "${APPS}/hexacopter-mission.desktop"
# GNOME requires desktop launchers to be marked trusted before they run.
gio set "${DESK}" metadata::trusted true 2>/dev/null || true
update-desktop-database "${APPS}" 2>/dev/null || true
echo "    Launcher on Desktop (double-click 'Hexacopter Mission')."

# --- 2. camera boot service --------------------------------------------------
echo "==> Installing ${SERVICE} (needs sudo)..."
sudo install -m 644 "${REPO}/systemd/${SERVICE}" "/etc/systemd/system/${SERVICE}"
sudo systemctl daemon-reload
sudo systemctl enable "${SERVICE}"
sudo systemctl restart "${SERVICE}"
sleep 3
echo
sudo systemctl --no-pager --lines=5 status "${SERVICE}" || true

echo
echo "Done."
echo "  - Camera publisher runs on every boot (systemctl status ${SERVICE})."
echo "  - Double-click the Desktop icon to open the mission menu."
