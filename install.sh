#!/usr/bin/env bash
# Install the two desktop shortcuts (no sudo, no systemd — user level only):
#   * Hexacopter Mission — opens the mission menu in a terminal
#   * AI Camera (toggle)  — starts/stops the OAK-D tracker on demand
#
# The AI camera is intentionally NOT a boot service: it is optional and toggled
# by hand, fully decoupled from the core MAVLink/telemetry background services.
#
# Run from your desktop terminal:  bash install.sh
set -euo pipefail

REPO="$(cd "$(dirname "$0")" && pwd)"
APPS="${HOME}/.local/share/applications"
DESKTOP_DIR="${HOME}/Desktop"
mkdir -p "${APPS}"

chmod +x "${REPO}/run_missions.sh" "${REPO}/ai_camera.sh"

install_launcher() {
    local file="$1"          # basename of the .desktop in the repo
    local dest="${DESKTOP_DIR}/${file}"
    install -m 755 "${REPO}/${file}" "${dest}"
    install -m 644 "${REPO}/${file}" "${APPS}/${file}"
    # GNOME requires desktop launchers to be marked trusted before they run.
    gio set "${dest}" metadata::trusted true 2>/dev/null || true
    echo "    installed ${file}"
}

echo "==> Installing desktop shortcuts..."
install_launcher "hexacopter-mission.desktop"
install_launcher "ai-camera-toggle.desktop"
update-desktop-database "${APPS}" 2>/dev/null || true

echo
echo "Done. On your Desktop:"
echo "  - 'Hexacopter Mission' opens the mission menu."
echo "  - 'AI Camera (toggle)' starts/stops the OAK-D tracker (toggle)."
echo
echo "The AI camera can also be driven from a terminal:"
echo "  ./ai_camera.sh start | stop | status | restart"
