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
