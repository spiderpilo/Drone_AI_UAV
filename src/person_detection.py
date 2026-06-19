#!/usr/bin/env python3
"""
Person detection and drone following via MAVLink.

Pipeline:
  USB camera -> YOLOv8n (ONNX/onnxruntime) -> person bbox
  -> proportional velocity controller -> MAVLink -> Pixhawk (GUIDED mode)

Hardware:
  - Raspberry Pi (aarch64, Ubuntu 26.04)
  - Pixhawk PX4 2.4.8 connected via USB (/dev/ttyACM0) or UART (/dev/ttyAMA0)
  - USB camera

Usage:
  python3 person_detection.py
  python3 person_detection.py --port /dev/ttyAMA0 --baud 57600 --camera 0
  python3 person_detection.py --dry-run   # camera + detection only, no flight commands
  python3 person_detection.py --stream    # live video at http://192.168.4.1:8080
"""

import argparse
import os
import sys
import threading
import time
from http.server import HTTPServer, BaseHTTPRequestHandler

import cv2
import numpy as np
import onnxruntime as ort
from pymavlink import mavutil

# ---------------------------------------------------------------------------
# Configuration (override via CLI args or environment variables)
# ---------------------------------------------------------------------------
DEFAULT_MODEL   = os.path.expanduser("~/drone/models/yolov8n.onnx")
DEFAULT_PORT    = os.environ.get("PIXHAWK_PORT", "/dev/ttyACM0")
DEFAULT_BAUD    = int(os.environ.get("PIXHAWK_BAUD", "57600"))
DEFAULT_CAMERA  = int(os.environ.get("DRONE_CAMERA", "0"))

# Detection
CONF_THRESHOLD  = 0.50   # minimum confidence to accept a detection
NMS_THRESHOLD   = 0.45   # NMS IoU threshold
PERSON_CLASS    = 0      # COCO class 0 = person
INPUT_SIZE      = 640    # YOLOv8 expects 640x640

# Control tuning  (adjust these during flight tests)
KP_YAW          = 0.6    # yaw rate gain  (rad/s per normalised pixel error)
KP_ALTITUDE     = 0.4    # vertical velocity gain (m/s per normalised error)
KP_DISTANCE     = 0.5    # forward/backward gain  (m/s per normalised error)
TARGET_SIZE     = 0.35   # desired person height as fraction of frame height
MAX_SPEED_H     = 0.8    # m/s horizontal clamp
MAX_SPEED_V     = 0.4    # m/s vertical clamp
CMD_RATE_HZ     = 10     # how often to send MAVLink velocity commands
PERSON_LOST_S   = 1.0    # seconds before issuing hover when person disappears

# MAVLink type_mask: use velocity (vx, vy, vz) + yaw_rate only
# Bit=1 means IGNORE that field.  bits[10..0]: yaw_rate, yaw, az, ay, ax, vz, vy, vx, pz, py, px
# Use vx(bit3), vy(bit4), vz(bit5), yaw_rate(bit10)  ->  ignore rest
_TYPE_MASK_VEL_YAW = (
    (1 << 0) | (1 << 1) | (1 << 2) |           # ignore position
    (1 << 6) | (1 << 7) | (1 << 8) |           # ignore acceleration
    (1 << 9)                                     # ignore yaw (use yaw_rate)
)  # = 967


# ---------------------------------------------------------------------------
# Person detector using YOLOv8n ONNX
# ---------------------------------------------------------------------------
class PersonDetector:
    def __init__(self, model_path: str):
        if not os.path.exists(model_path):
            sys.exit(f"[detector] Model not found: {model_path}\n"
                     f"  Run: python3 -c \"import urllib.request; "
                     f"urllib.request.urlretrieve('https://github.com/ultralytics/assets/"
                     f"releases/download/v8.4.0/yolov8n.onnx', '{model_path}')\"")

        self._sess = ort.InferenceSession(
            model_path,
            providers=["CPUExecutionProvider"]
        )
        self._in_name  = self._sess.get_inputs()[0].name
        self._out_name = self._sess.get_outputs()[0].name
        print(f"[detector] Loaded {model_path}")

    def detect(self, frame: np.ndarray) -> list[dict]:
        """
        Returns list of { 'bbox': (x, y, w, h), 'conf': float } for every person
        detected above CONF_THRESHOLD in the given BGR frame.
        """
        h, w = frame.shape[:2]
        img, scale, pad_x, pad_y = self._preprocess(frame, w, h)

        raw = self._sess.run([self._out_name], {self._in_name: img})[0]
        # YOLOv8 ONNX output shape: (1, 84, 8400)
        # 84 = cx, cy, bw, bh + 80 class scores
        preds = raw[0].T  # -> (8400, 84)

        boxes, scores = [], []
        for pred in preds:
            cls_scores = pred[4:]
            cls_id = int(np.argmax(cls_scores))
            conf   = float(cls_scores[cls_id])
            if cls_id != PERSON_CLASS or conf < CONF_THRESHOLD:
                continue
            cx, cy, bw, bh = pred[:4]
            x1 = (cx - bw / 2 - pad_x) / scale
            y1 = (cy - bh / 2 - pad_y) / scale
            boxes.append([float(x1), float(y1), float(bw / scale), float(bh / scale)])
            scores.append(conf)

        if not boxes:
            return []

        indices = cv2.dnn.NMSBoxes(boxes, scores, CONF_THRESHOLD, NMS_THRESHOLD)
        results = []
        for i in (indices.flatten() if hasattr(indices, "flatten") else indices):
            x, y, bw, bh = boxes[i]
            results.append({
                "bbox": (int(x), int(y), int(bw), int(bh)),
                "conf": scores[i],
            })
        return results

    @staticmethod
    def _preprocess(frame, w, h):
        scale  = INPUT_SIZE / max(w, h)
        nw, nh = int(w * scale), int(h * scale)
        resized = cv2.resize(frame, (nw, nh))
        canvas  = np.full((INPUT_SIZE, INPUT_SIZE, 3), 114, dtype=np.uint8)
        pad_x   = (INPUT_SIZE - nw) // 2
        pad_y   = (INPUT_SIZE - nh) // 2
        canvas[pad_y:pad_y + nh, pad_x:pad_x + nw] = resized
        blob = canvas.astype(np.float32) / 255.0
        blob = blob.transpose(2, 0, 1)[np.newaxis]   # HWC -> 1CHW
        return blob, scale, pad_x, pad_y


# ---------------------------------------------------------------------------
# MAVLink drone interface
# ---------------------------------------------------------------------------
class DroneController:
    def __init__(self, port: str, baud: int):
        print(f"[mavlink] Connecting to {port} @ {baud} baud …")
        self.master = mavutil.mavlink_connection(port, baud=baud)
        hb = self.master.wait_heartbeat(timeout=10)
        if hb is None:
            raise TimeoutError(f"No heartbeat from Pixhawk on {port}")
        self.sys  = self.master.target_system
        self.comp = self.master.target_component
        print(f"[mavlink] Heartbeat: system={self.sys} component={self.comp}")

    def set_guided_mode(self):
        """Request GUIDED mode (ArduCopter). For PX4 firmware use OFFBOARD instead."""
        self.master.mav.command_long_send(
            self.sys, self.comp,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            4,   # ArduCopter GUIDED = 4
            0, 0, 0, 0, 0,
        )
        print("[mavlink] Requested GUIDED mode")

    def send_velocity(self, vx: float, vy: float, vz: float, yaw_rate: float = 0.0):
        """
        Send body-frame velocity command.
        vx = forward (+) / backward (-)  m/s
        vy = right (+) / left (-)         m/s
        vz = down (+) / up (-)            m/s  (NED convention)
        yaw_rate = clockwise (+) rad/s
        """
        self.master.mav.set_position_target_local_ned_send(
            int(time.monotonic() * 1000) & 0xFFFF_FFFF,
            self.sys, self.comp,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            _TYPE_MASK_VEL_YAW,
            0, 0, 0,              # position (ignored)
            vx, vy, vz,           # velocity m/s
            0, 0, 0,              # acceleration (ignored)
            0, yaw_rate,          # yaw (ignored), yaw_rate rad/s
        )

    def stop(self):
        self.send_velocity(0.0, 0.0, 0.0, 0.0)


# ---------------------------------------------------------------------------
# Proportional controller
# ---------------------------------------------------------------------------
def compute_velocity(det: dict, frame_w: int, frame_h: int) -> tuple[float, float, float, float]:
    """
    Given a person detection and frame dimensions, return (vx, vy, vz, yaw_rate).

    Strategy:
      - yaw_rate: rotate drone so person stays horizontally centred
      - vz:       adjust altitude so person stays vertically centred
      - vx:       approach or retreat to maintain TARGET_SIZE bounding-box ratio
      - vy:       zero (yaw handles lateral tracking; strafe reserved for fine tuning)
    """
    x, y, bw, bh = det["bbox"]
    cx_norm = (x + bw / 2) / frame_w - 0.5    # -0.5 (left) .. +0.5 (right)
    cy_norm = (y + bh / 2) / frame_h - 0.5    # -0.5 (top)  .. +0.5 (bottom)
    size_err = (bh / frame_h) - TARGET_SIZE    # positive = too close

    yaw_rate = KP_YAW      * cx_norm
    vz       = KP_ALTITUDE * cy_norm           # positive cy_norm -> person below -> descend
    vx       = KP_DISTANCE * size_err          # positive -> too close -> move backward... wait
    # size_err > 0 means person is LARGER than target (too close) -> retreat (vx negative)
    vx       = -KP_DISTANCE * size_err
    vy       = 0.0

    vx  = float(np.clip(vx,       -MAX_SPEED_H, MAX_SPEED_H))
    vy  = float(np.clip(vy,       -MAX_SPEED_H, MAX_SPEED_H))
    vz  = float(np.clip(vz,       -MAX_SPEED_V, MAX_SPEED_V))
    yaw_rate = float(np.clip(yaw_rate, -1.5, 1.5))

    return vx, vy, vz, yaw_rate


# ---------------------------------------------------------------------------
# MJPEG streaming server (--stream flag)
# ---------------------------------------------------------------------------
STREAM_PORT = 8080

class _StreamServer:
    """Serves the latest annotated frame as an MJPEG stream over HTTP."""

    def __init__(self, port: int = STREAM_PORT):
        self._frame = None
        self._lock = threading.Lock()
        self._port = port

        parent = self

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                self.send_response(200)
                self.send_header("Content-Type",
                                 "multipart/x-mixed-replace; boundary=frame")
                self.end_headers()
                while True:
                    with parent._lock:
                        jpg = parent._frame
                    if jpg is None:
                        time.sleep(0.05)
                        continue
                    try:
                        self.wfile.write(b"--frame\r\n")
                        self.wfile.write(b"Content-Type: image/jpeg\r\n\r\n")
                        self.wfile.write(jpg)
                        self.wfile.write(b"\r\n")
                    except BrokenPipeError:
                        break
                    time.sleep(0.05)

            def log_message(self, *_):
                pass

        self._httpd = HTTPServer(("0.0.0.0", self._port), Handler)
        t = threading.Thread(target=self._httpd.serve_forever, daemon=True)
        t.start()
        print(f"[stream] Live video at http://192.168.4.1:{self._port}")

    def update(self, frame: np.ndarray):
        _, jpg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
        with self._lock:
            self._frame = jpg.tobytes()


def _annotate_frame(frame, detections, best, vx, vz, yr):
    """Draw bounding boxes and control info on the frame."""
    fh, fw = frame.shape[:2]

    # Crosshair at frame centre
    cv2.line(frame, (fw // 2 - 20, fh // 2), (fw // 2 + 20, fh // 2), (0, 255, 0), 1)
    cv2.line(frame, (fw // 2, fh // 2 - 20), (fw // 2, fh // 2 + 20), (0, 255, 0), 1)

    for det in detections:
        x, y, bw, bh = det["bbox"]
        color = (0, 255, 0) if det is best else (200, 200, 200)
        cv2.rectangle(frame, (x, y), (x + bw, y + bh), color, 2)
        cv2.putText(frame, f"{det['conf']:.0%}", (x, y - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

    if best:
        cv2.putText(frame, f"vx={vx:+.2f} vz={vz:+.2f} yr={yr:+.2f}",
                    (10, fh - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    else:
        cv2.putText(frame, "NO PERSON", (10, fh - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    return frame


# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------
def run(args):
    detector = PersonDetector(args.model)

    drone = None
    if not args.dry_run:
        try:
            drone = DroneController(args.port, args.baud)
            drone.set_guided_mode()
        except Exception as exc:
            print(f"[mavlink] WARNING: {exc}")
            print("[mavlink] Continuing in camera-only mode (no flight commands)")

    stream = _StreamServer() if args.stream else None

    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        sys.exit(f"[camera] Cannot open camera index {args.camera}")

    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    fw = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    fh = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"[camera] Opened at {fw}x{fh}")

    cmd_interval   = 1.0 / CMD_RATE_HZ
    last_cmd_t     = 0.0
    last_person_t  = 0.0
    hovering       = True

    print("[main] Running — press Ctrl+C to stop")
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("[camera] Frame read failed, retrying…")
                time.sleep(0.05)
                continue

            detections = detector.detect(frame)
            now = time.monotonic()

            best = None
            vx = vy = vz = yr = 0.0

            if detections:
                last_person_t = now
                hovering = False

                if now - last_cmd_t >= cmd_interval:
                    best = max(detections, key=lambda d: d["bbox"][2] * d["bbox"][3])
                    vx, vy, vz, yr = compute_velocity(best, fw, fh)

                    x, y, bw, bh = best["bbox"]
                    print(
                        f"[detect] person ({x+bw//2:3d},{y+bh//2:3d}) "
                        f"conf={best['conf']:.2f} "
                        f"vx={vx:+.2f} vz={vz:+.2f} yr={yr:+.2f}"
                    )

                    if drone:
                        drone.send_velocity(vx, vy, vz, yr)
                    last_cmd_t = now

            else:
                if not hovering and (now - last_person_t) > PERSON_LOST_S:
                    print("[detect] Person lost — hovering")
                    if drone:
                        drone.stop()
                    hovering = True

            if stream:
                annotated = _annotate_frame(frame.copy(), detections, best, vx, vz, yr)
                stream.update(annotated)

    except KeyboardInterrupt:
        print("\n[main] Interrupted")
    finally:
        if drone:
            drone.stop()
            print("[mavlink] Stop command sent")
        cap.release()
        print("[main] Done")


# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description="Drone person-following via MAVLink")
    parser.add_argument("--model",   default=DEFAULT_MODEL,  help="Path to yolov8n.onnx")
    parser.add_argument("--port",    default=DEFAULT_PORT,   help="Pixhawk serial port")
    parser.add_argument("--baud",    default=DEFAULT_BAUD,   type=int)
    parser.add_argument("--camera",  default=DEFAULT_CAMERA, type=int, help="Camera index")
    parser.add_argument("--dry-run", action="store_true",
                        help="Run detection only, send no flight commands")
    parser.add_argument("--stream",  action="store_true",
                        help="Serve live MJPEG video at http://192.168.4.1:8080")
    run(parser.parse_args())


if __name__ == "__main__":
    main()
