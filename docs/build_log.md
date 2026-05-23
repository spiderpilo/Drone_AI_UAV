# Build Log

### May 12, 2026
- Assembled Drone Frame. 
- Attached Motors and ESC's

### May 20, 2026 
- Set up Raspberry Pi with Linux Ubuntu
- Set up SSH for Raspberry Pi

### May 21, 2026
- Installed OpenCV (4.10.0), onnxruntime (1.26.0), pymavlink (2.4.49) on Pi
- Downloaded YOLOv8n ONNX model (~13 MB) to ~/drone/models/yolov8n.onnx
- Created person_detection.py: YOLOv8n person detection + MAVLink velocity commands to Pixhawk
- Configured Pi as WiFi access point (SSID: DroneAI, IP: 192.168.4.1)
  - Auto-starts on boot via NetworkManager — no router needed in the field
  - Connect: WiFi → DroneAI / Bullet1234, then ssh piolo@192.168.4.1

### May 22, 2026
- Soldered ESC's (electronic speed controllers) to PDB (power distribution board)
- 3D printed custom mounts
- Mounted Webcam

### May 23, 2026
- Mounted Raspberry Pi 4 onto frame
- Motor Tests: ALL PASS
- Reciever and transmitter pairing and testing
