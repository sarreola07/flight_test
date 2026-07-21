# Rebuilding on a fresh Jetson (after NVMe / JetPack reinstall)

This repo does **not** contain the camera environment or the Python virtualenvs —
they live outside it and are lost when the Jetson is wiped. Follow these steps on
the fresh install to get everything working again.

## What survives the wipe vs. what you rebuild

| Item | Survives? | Notes |
|---|---|---|
| `flight_test` repo | ✅ on GitHub | `git clone` it back |
| PX4 parameters (`COM_CPU_MAX=-1`, `EKF2_AID_MASK=1`, airframe) | ✅ | Stored on the Pixhawk FMU, not the Jetson |
| Mission venv (`venv/`) | ❌ rebuild | `setup.sh` recreates it |
| Camera env (`~/oak_drone_project/`, 588 MB) | ❌ rebuild | steps below |
| OAK-D udev rule (`80-movidius.rules`) | ❌ rebuild | steps below |
| `hexacopter-follow/` | ✅ its own remote | re-clone (optional) |

## 1. Clone the project and build the mission venv

```bash
git clone https://github.com/sarreola07/flight_test.git ~/Desktop/pixhawk-test
cd ~/Desktop/pixhawk-test
bash setup.sh          # venv + pymavlink + pyserial + dialout access
./venv/bin/pip install pillow   # only needed if you regenerate the icons
```

## 2. Rebuild the OAK-D camera environment

```bash
mkdir -p ~/oak_drone_project && cd ~/oak_drone_project
git clone https://github.com/luxonis/depthai-python.git
python3 -m venv depthai-env
./depthai-env/bin/pip install --upgrade pip
./depthai-env/bin/pip install depthai==2.32.0.0 opencv-python==4.13.0.92 numpy==2.5.1

# Download the example model blobs (incl. mobilenet-ssd person detector)
cd depthai-python/examples
../../depthai-env/bin/python install_requirements.py
```

Confirm the blob `camera_publisher.py` expects now exists:
`~/oak_drone_project/depthai-python/examples/models/mobilenet-ssd_openvino_2021.4_6shave.blob`

## 3. OAK-D udev rule (non-root USB access)

```bash
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' \
  | sudo tee /etc/udev/rules.d/80-movidius.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Verify the camera is seen:
```bash
~/oak_drone_project/depthai-env/bin/python - <<'PY'
import depthai as dai
print([d.getMxId() for d in dai.Device.getAllAvailableDevices()])
PY
```

## 4. Desktop shortcuts

```bash
cd ~/Desktop/pixhawk-test && bash install.sh
```

Installs the **Hexacopter Mission** and **AI Camera (toggle)** Desktop icons
(user level, no sudo).

## 5. (Optional) the classmates' ArduPilot SITL project

```bash
git clone https://github.com/robertonava08/hexacopter-follow.git \
  ~/Desktop/pixhawk-test/hexacopter-follow
```

It is git-ignored by this repo, so it stays a separate checkout.

## Sanity check

```bash
cd ~/Desktop/pixhawk-test
./venv/bin/python check_pixhawk.py     # MAVLink link to the Pixhawk
./ai_camera.sh start && sleep 10 && ./ai_camera.sh status && ./ai_camera.sh stop
```
