# System Architecture

## Overview

This project is an AI-assisted UAV platform designed to combine autonomous flight control, edge computer vision, and modular hardware expansion.

The system uses a Pixhawk flight controller for low-level flight stabilization and safety-critical control, while a Raspberry Pi handles higher-level AI tasks such as person detection, target tracking, and future autonomous decision-making.

The goal is not to replace the flight controller, but to build an intelligent companion system around it.

---

## High-Level Architecture

text
USB Camera
    ↓
Raspberry Pi 4
    ↓
Computer Vision / AI Detection
    ↓
Tracking Logic
    ↓
MAVLink Communication
    ↓
Pixhawk Flight Controller
    ↓
ESCs
    ↓
Brushless Motors

# Core Components

## Raspberry Pi 4

The Raspberry Pi acts as the onboard companion computer.

### Responsibilities:

Run computer vision models
Process camera input
Detect and track people or objects
Send high-level movement commands to the Pixhawk
Handle future AI features such as fire detection or threat detection

The Pi does not directly control the motors. Instead, it communicates with the Pixhawk.

## Pixhawk Flight Controller

The Pixhawk handles the critical flight-control layer.

### Responsibilities:

Stabilization
Motor mixing
ESC control
GPS-based positioning
Flight modes
Failsafe behavior
RC control override

The Pixhawk remains responsible for safe flight behavior.

## USB Camera

The USB camera provides the visual input for the AI system.

### Responsibilities:

Capture live video frames
Provide input to the person/object detection model
Allow the Raspberry Pi to estimate target position in the frame
AI Vision Module

The AI vision module processes camera frames and detects objects.

# Initial goal:

Detect a person using a pretrained object detection model

# Future goals:

Fire detection
Threat detection
Multi-object tracking
Search and rescue support
Tracking Module

The tracking module converts AI detection results into movement intent.

Example:

Person is left of center → rotate or move left
Person is right of center → rotate or move right
Person is too small → move forward
Person is too large → move backward or hold position

This module produces high-level movement commands, not direct motor commands.

## MAVLink Communication Layer

MAVLink is used for communication between the Raspberry Pi and Pixhawk.

### Responsibilities:

Send guided movement commands
Read telemetry data
Monitor drone state
Check arming status
Detect failsafe conditions

Example data exchanged:

Drone altitude
GPS position
Battery status
Flight mode
Target movement commands
Data Flow
1. Camera captures video frame
2. Raspberry Pi reads frame
3. AI model detects target
4. Tracking module calculates target offset
5. Raspberry Pi sends movement command through MAVLink
6. Pixhawk executes safe flight adjustment
7. Pixhawk continues stabilization and failsafe monitoring
Safety Design

The system is designed with the Pixhawk as the primary safety controller.

# Safety principles:

Manual RC control has priority
Pixhawk handles stabilization
Pixhawk handles failsafes
Raspberry Pi only sends high-level commands
Autonomous behavior can be disabled
Low battery should trigger landing or RTL
Loss of communication should stop autonomous movement
Flight Control Philosophy

This project separates flight control into two layers:

### Low-Level Control

Handled by Pixhawk.

Includes:

Motor control
Stabilization
ESC output
Attitude control
GPS hold
Return-to-launch behavior
High-Level Autonomy

### Handled by Raspberry Pi.

Includes:

Person detection
Tracking decisions
AI-based perception
Future mission logic

This separation makes the system safer and easier to debug.

Current Development Stage

# Current focus:

Build and document the UAV hardware platform
Set up Pixhawk-based flight control
Connect Raspberry Pi as a companion computer
Run person detection using a USB camera
Test tracking logic without full autonomous flight first
Planned Architecture Improvements

# Future upgrades may include:

Jetson or Coral TPU for faster AI inference
Thermal camera for fire detection
Custom PCB for power distribution and sensor integration
ROS2 integration
Obstacle avoidance sensors
Better telemetry dashboard
Autonomous mission planning
Onboard logging system
Design Reasoning

This architecture was chosen because it allows the project to grow safely.

Instead of trying to build a flight controller from scratch, the Pixhawk handles proven flight-control responsibilities. This lets the Raspberry Pi focus on AI, perception, and higher-level decision-making.

This makes the project more realistic, modular, and closer to how real robotics systems are designed.