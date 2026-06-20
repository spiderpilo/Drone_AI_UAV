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
