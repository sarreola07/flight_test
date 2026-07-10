#!/usr/bin/env bash
# One-time setup for the Pixhawk connection test.
# Run with:  bash setup.sh
set -e
cd "$(dirname "$0")"

echo "==> Adding $USER to the 'dialout' group (needed for /dev/ttyACM0)..."
sudo usermod -aG dialout "$USER"

echo "==> Granting immediate access to /dev/ttyACM0 for this session..."
sudo setfacl -m "u:$USER:rw" /dev/ttyACM0 2>/dev/null || sudo chmod a+rw /dev/ttyACM0

echo "==> Creating Python virtual environment..."
python3 -m venv venv

echo "==> Installing pymavlink..."
./venv/bin/pip install --upgrade pip -q
./venv/bin/pip install -r requirements.txt -q

echo
echo "Done. Run the test with:"
echo "    ./venv/bin/python check_pixhawk.py"
echo
echo "Note: the 'dialout' group membership becomes permanent after you log out"
echo "and back in (or reboot). Until then the setfacl above covers this session."
