# AI Vision Follow Drone

An autonomous drone system that uses computer vision to detect and follow a target while maintaining a safe, configurable distance.  
This project focuses on real-time perception, control feedback, and hardware–software integration for autonomous flight.

---

## 🚁 Project Overview

The goal of **AI Vision Follow Drone** is to design and build an autonomous drone capable of:

- Detecting a human target using computer vision
- Tracking the target in real time
- Maintaining a consistent following distance
- Adapting movement based on the target’s position and motion
- Operating safely with hardware constraints and fail-safes

This project is built as a **full-stack robotics system**, combining AI perception, control logic, embedded hardware, and mechanical design.

---

## Raspberry Pi SSH

The Pi runs as its own WiFi hotspot — no external router needed.

1. Power on the Pi and wait ~30 seconds for boot
2. Connect to WiFi: **DroneAI** / password: **Bullet1234**
3. SSH in: `ssh piolo@192.168.4.1`

### Running the detection script

```bash
# Dry run (camera + detection only, no flight commands)
python3 ~/drone/person_detection.py --dry-run

# With live video stream (open http://192.168.4.1:8080 in browser)
python3 ~/drone/person_detection.py --dry-run --stream

# Full run with Pixhawk connected
python3 ~/drone/person_detection.py --port /dev/ttyACM0 --stream
```

### Updating the software

After editing code locally in VSCode, push your changes and pull them on the Pi:

**On your computer (VSCode terminal):**
```bash
git add -A && git commit -m "your message" && git push
```

**On the Pi (SSH session):**
```bash
cd ~/drone
git pull
```

The updated code is now ready to run on the Pi.

---

## Custom Model Training

Fine-tune YOLOv8n on person detection data and deploy via Docker.

### 1. Train the model (on your Mac)

```bash
pip install -r training/requirements.txt
python3 training/train.py --epochs 30
```

The trained model is saved to `training/output/person_detector.onnx`.

### 2. Build and deploy with Docker

Copy the trained model into the Docker build context, then deploy to the Pi:

```bash
cp training/output/person_detector.onnx docker/models/
./docker/deploy.sh
```

### 3. Run on the Pi

```bash
# Dry run (camera only)
docker run --rm --device /dev/video0 drone-vision --dry-run

# With Pixhawk + live stream
docker run --rm --device /dev/video0 --device /dev/ttyACM0 -p 8080:8080 drone-vision --port /dev/ttyACM0
```

> **Note:** Docker must be installed on the Pi first:
> ```bash
> curl -fsSL https://get.docker.com | sh
> sudo usermod -aG docker piolo
> ```
