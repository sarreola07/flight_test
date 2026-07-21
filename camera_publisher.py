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

import argparse
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


def build_pipeline(blob_path, preview=False):
    """RGB + stereo-depth + MobileNet spatial detection pipeline.

    preview=True also streams the RGB frames out so a window can be drawn.
    """
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

    if preview:
        xout_rgb = pipeline.create(dai.node.XLinkOut)
        xout_rgb.setStreamName("rgb")
        # passthrough gives the RGB frame the detections were computed on (synced)
        spatial_nn.passthrough.link(xout_rgb.input)
    return pipeline


def nearest_person(detections):
    """Return the closest person detection (smallest Z), or None."""
    people = [d for d in detections if d.label == PERSON_LABEL]
    if not people:
        return None
    return min(people, key=lambda d: d.spatialCoordinates.z or float("inf"))


def _draw_preview(cv2, frame, detections):
    """Draw person boxes + X/Y/Z distance onto the RGB frame (in place)."""
    h, w = frame.shape[:2]
    for det in detections:
        if det.label != PERSON_LABEL:
            continue
        x1, y1 = int(det.xmin * w), int(det.ymin * h)
        x2, y2 = int(det.xmax * w), int(det.ymax * h)
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        zx = det.spatialCoordinates.x / 1000.0
        zy = det.spatialCoordinates.y / 1000.0
        zz = det.spatialCoordinates.z / 1000.0
        cv2.putText(frame, "person {:.0f}%".format(det.confidence * 100),
                    (x1 + 6, y1 + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        for i, txt in enumerate(("X {:+.2f} m".format(zx), "Y {:+.2f} m".format(zy),
                                 "Z {:.2f} m".format(zz))):
            cv2.putText(frame, txt, (x1 + 6, y1 + 40 + i * 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)


def run_once(sock, blob_path, preview=False):
    """Open the camera and stream until the device drops or (preview) the user quits.

    Returns True if the user asked to quit (pressed q in the preview window).
    """
    cv2 = None
    if preview:
        import cv2  # only needed for the window; lives in the DepthAI env

    pipeline = build_pipeline(blob_path, preview=preview)
    with dai.Device(pipeline) as device:
        log("OAK-D connected (MxId {}). Publishing to {}:{}{}".format(
            device.getMxId(), UDP_IP, UDP_PORT, "  [preview]" if preview else ""))
        det_queue = device.getOutputQueue(name="detections", maxSize=4, blocking=False)
        rgb_queue = device.getOutputQueue(name="rgb", maxSize=4, blocking=False) if preview else None
        win = "AI Camera - person tracking (press q to close)"
        last_report = 0.0
        while True:
            in_det = det_queue.get()  # blocks until a frame arrives
            detections = in_det.detections

            person = nearest_person(detections)
            if person is not None:
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

            if preview:
                in_rgb = rgb_queue.get()
                frame = in_rgb.getCvFrame()
                _draw_preview(cv2, frame, detections)
                cv2.imshow(win, frame)
                if cv2.waitKey(1) == ord("q"):
                    cv2.destroyAllWindows()
                    return True


def main():
    parser = argparse.ArgumentParser(description="OAK-D person tracker -> UDP")
    parser.add_argument("--preview", action="store_true",
                        help="show a live window with person boxes (needs a display)")
    args = parser.parse_args()

    if not Path(BLOB_PATH).exists():
        sys.exit("Model blob not found: {}".format(BLOB_PATH))
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    log("Camera publisher starting{}. Blob: {}".format(
        " (preview)" if args.preview else "", BLOB_PATH))
    while True:
        try:
            if run_once(sock, BLOB_PATH, preview=args.preview):
                log("Preview closed — camera stopped.")
                return
        except KeyboardInterrupt:
            log("Camera publisher stopped.")
            return
        except Exception as exc:
            log("Camera error: {} — reconnecting in {}s".format(exc, RECONNECT_WAIT_S))
            time.sleep(RECONNECT_WAIT_S)


if __name__ == "__main__":
    main()
