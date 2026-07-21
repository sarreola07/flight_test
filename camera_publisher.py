#!/usr/bin/env python3
"""
OAK-D person tracker -> UDP publisher.

Runs the OAK-D spatial person detector headlessly and broadcasts the nearest
person's position as JSON over UDP, so the mission app (in a separate venv) can
read it without importing DepthAI.

  Sends, per detection:  {"x": <m>, "y": <m>, "z": <m>, "conf": <0..1>}
  X = left(-)/right(+),  Y = down(-)/up(+),  Z = forward distance, all in metres,
  relative to the camera.

IMPORTANT: run this with the DepthAI environment, not the mission venv:
    ~/oak_drone_project/depthai-env/bin/python camera_publisher.py

The oak-camera systemd service starts this automatically on boot.
"""

import json
import os
import socket
import sys
import time
from pathlib import Path

try:
    import depthai as dai
except ImportError:
    sys.exit("depthai not found. Run with ~/oak_drone_project/depthai-env/bin/python")

# --- config (override via env) ------------------------------------------------
UDP_IP = os.environ.get("CAMERA_UDP_IP", "127.0.0.1")
UDP_PORT = int(os.environ.get("CAMERA_UDP_PORT", "5005"))
CONFIDENCE = float(os.environ.get("CAMERA_CONFIDENCE", "0.5"))
PERSON_LABEL = 15  # MobileNet-SSD: index 15 == "person"
DEFAULT_BLOB = str(Path.home() / "oak_drone_project/depthai-python/examples/"
                   "models/mobilenet-ssd_openvino_2021.4_6shave.blob")
BLOB_PATH = os.environ.get("CAMERA_BLOB", DEFAULT_BLOB)
RECONNECT_WAIT_S = 5
# -----------------------------------------------------------------------------


def log(msg):
    print(msg, flush=True)


def build_pipeline(blob_path):
    """Headless RGB + stereo-depth + MobileNet spatial detection pipeline."""
    pipeline = dai.Pipeline()

    cam_rgb = pipeline.create(dai.node.ColorCamera)
    mono_left = pipeline.create(dai.node.MonoCamera)
    mono_right = pipeline.create(dai.node.MonoCamera)
    stereo = pipeline.create(dai.node.StereoDepth)
    spatial_nn = pipeline.create(dai.node.MobileNetSpatialDetectionNetwork)
    xout_nn = pipeline.create(dai.node.XLinkOut)
    xout_nn.setStreamName("detections")

    cam_rgb.setPreviewSize(300, 300)
    cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    cam_rgb.setInterleaved(False)
    cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

    mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    mono_left.setCamera("left")
    mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    mono_right.setCamera("right")

    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
    stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
    stereo.setSubpixel(True)
    stereo.setOutputSize(mono_left.getResolutionWidth(), mono_left.getResolutionHeight())

    spatial_nn.setBlobPath(blob_path)
    spatial_nn.setConfidenceThreshold(CONFIDENCE)
    spatial_nn.input.setBlocking(False)
    spatial_nn.setBoundingBoxScaleFactor(0.5)
    spatial_nn.setDepthLowerThreshold(100)     # mm
    spatial_nn.setDepthUpperThreshold(10000)   # mm

    mono_left.out.link(stereo.left)
    mono_right.out.link(stereo.right)
    cam_rgb.preview.link(spatial_nn.input)
    stereo.depth.link(spatial_nn.inputDepth)
    spatial_nn.out.link(xout_nn.input)
    return pipeline


def nearest_person(detections):
    """Return the closest person detection (smallest Z), or None."""
    people = [d for d in detections if d.label == PERSON_LABEL]
    if not people:
        return None
    return min(people, key=lambda d: d.spatialCoordinates.z or float("inf"))


def run_once(sock, blob_path):
    """Open the camera and stream until the device drops or an error occurs."""
    pipeline = build_pipeline(blob_path)
    with dai.Device(pipeline) as device:
        log("OAK-D connected (MxId {}). Publishing to {}:{}".format(
            device.getMxId(), UDP_IP, UDP_PORT))
        det_queue = device.getOutputQueue(name="detections", maxSize=4, blocking=False)
        last_report = 0.0
        while True:
            in_det = det_queue.get()  # blocks until a frame arrives
            person = nearest_person(in_det.detections)
            if person is None:
                continue
            payload = {
                "x": person.spatialCoordinates.x / 1000.0,
                "y": person.spatialCoordinates.y / 1000.0,
                "z": person.spatialCoordinates.z / 1000.0,
                "conf": float(person.confidence),
            }
            sock.sendto(json.dumps(payload).encode(), (UDP_IP, UDP_PORT))
            now = time.time()
            if now - last_report >= 1.0:   # throttle console logging to 1 Hz
                log("person X:{x:+.2f} Y:{y:+.2f} Z:{z:.2f} m  ({conf:.0%})".format(**payload))
                last_report = now


def main():
    if not Path(BLOB_PATH).exists():
        sys.exit("Model blob not found: {}".format(BLOB_PATH))
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    log("Camera publisher starting. Blob: {}".format(BLOB_PATH))
    while True:
        try:
            run_once(sock, BLOB_PATH)
        except KeyboardInterrupt:
            log("Camera publisher stopped.")
            return
        except Exception as exc:
            log("Camera error: {} — reconnecting in {}s".format(exc, RECONNECT_WAIT_S))
            time.sleep(RECONNECT_WAIT_S)


if __name__ == "__main__":
    main()
