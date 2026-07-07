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


### May 29, 2026
- Calibrated Pixhawk 
- Removed Webcam and Raspberry Pi for flight testing

### June 8, 2026
- Drone ARMED successfully via channel 2
- First Test Flight: Drone tilts on one side before take off

### June 9, 2026
- Recalibrated Drone on QC ground control
- Fixed motor Orientation 
- Test Flight 2: Drone lifted from ground Succesfully 

### June 10, 2026
- Test flight 3: Drone succesfully leaves ground 
- Drone appears to be difficult to control and hard to maintain a fixed height

### July 7, 2026
- Re-Alligned motors and propllers
- Added Loiter function to the Controller.(Maintain Drone postion)
