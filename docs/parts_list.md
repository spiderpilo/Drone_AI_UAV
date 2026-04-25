# 🛸 Hexacopter Drone Build Parts List (600mm Frame)

## 📌 Overview
This build is based on a 600mm hexacopter frame designed for stability, payload capacity, and future expansion (AI / sensors / Jetson).

---

## 🧠 Core Components

### 🧩 Frame
- X600 600mm Carbon Fiber Hexacopter Frame

---

### ⚙️ Motors (x6)
- 2216 Brushless Motors  
- ~880KV (700–1000KV range acceptable)

**Notes:**
- Lower KV = more torque (better for larger props)
- Ensure all motors are identical

---

### ⚡ ESCs (x6)
- 30A–40A Electronic Speed Controllers
- Firmware: BLHeli_S or BLHeli_32

**Notes:**
- Must match motor current requirements
- One ESC per motor

---

### 🌀 Propellers (x6 total)
- Size: 10x4.5 or 12x4.5
- Configuration:
  - 3 Clockwise (CW)
  - 3 Counter-Clockwise (CCW)

**Notes:**
- Larger props = more lift, better efficiency
- Start with 10” for easier tuning

---

### 🔋 Battery
- 4S LiPo (14.8V)
- Capacity: 5000–8000mAh
- Connector: XT60

**Notes:**
- Start with 4S (simpler + safer than 6S)
- Higher mAh = longer flight time (but heavier)

---

### 🔌 Power Distribution
- Power Distribution Board (PDB) or integrated frame PDB
- XT60 connector

---

### 🎮 Flight Controller (Already Owned)
- Compatible with hexacopter configuration

---

## 🔧 Supporting Components

### 🔋 Battery Charging
- LiPo Balance Charger (e.g., iMAX B6)
- Power supply (to power charger)
- LiPo Safe Bag

---

### 📡 Receiver & Transmitter
- Radio transmitter (controller)
- Compatible receiver

---

### 🧠 Optional (Future Expansion)
- Raspberry Pi 4 (control / integration)
- NVIDIA Jetson (AI / computer vision)
- Camera module
- IMU / sensors

---

## ⚠️ Safety & Essentials
- LiPo safe storage bag
- Voltage checker / battery alarm
- Proper connectors (XT60, bullet connectors if needed)
- Heat shrink + soldering tools

---

## 🧪 First Build Recommendations
- Start with 10” props
- Use 4S battery
- Test motors individually before full assembly
- Verify ESC signal from flight controller before attaching props

---

## 🚀 Future Upgrades
- Switch to 6S battery for higher efficiency
- Add onboard AI (Jetson)
- Implement object tracking / autonomy
- Upgrade props for payload optimization

---