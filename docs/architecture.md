# System Architecture

## Overview
This project implements an AI-powered quadcopter capable of detecting and following a target using onboard computer vision.

## High-Level Flow
Camera → Raspberry Pi → Arduino → ESC → Motors

## Components

### Raspberry Pi
- Runs computer vision and AI inference
- Sends high-level movement commands

### Arduino
- Handles low-level motor and sensor control
- Receives commands over serial

### Flight Controller / ESC
- Stabilization and motor driving

## Communication
- Raspberry Pi ↔ Arduino: Serial
- Arduino → ESC: PWM

## Future Expansion
- Add obstacle avoidance
- Add GPS navigation
