#!/usr/bin/env bash
# Launcher used by the desktop icon: run the mission menu in this repo's venv,
# then hold the terminal open so results stay visible.
cd "$(dirname "$0")"

if [[ ! -x ./venv/bin/python ]]; then
    echo "venv not found. Run: bash setup.sh"
    read -r -n1 -p "Press any key to close..."
    exit 1
fi

./venv/bin/python missions.py
echo
read -r -n1 -p "Mission ended — press any key to close this window..."
