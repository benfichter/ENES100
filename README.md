ENES100: Outdoor Terrain Vehicle (OTV)

Overview
This repository contains the firmware and supporting code for our Outdoor Terrain Vehicle (OTV), designed for the ENES100: Introduction to Engineering Design course at the University of Maryland, College Park.

Our OTV is designed to autonomously navigate outdoor terrain, perform mission objectives, and transmit data using an integrated sensor suite. The project combines mechanical design, materials selection, embedded programming, and teamwork across multiple subsystems.

Repository Structure
main.ino            # Core Arduino program for the OTV
/drive              # Motor control, PID tuning, and movement algorithms
/components         # Sensor integration, servo control, and I/O management
/docs               # Reports, design files, and documentation (optional)
README.md           # This file

Features
- Autonomous navigation via waypoint-based control
- Sensor integration (ultrasonic, load cell, IR)
- Custom drive algorithms with PID correction
- Modular component system for easy debugging and updates
- Lightweight frame optimized by the Materials Team

Hardware Overview
- Microcontroller: Arduino Mega 2560
- Motors: Dual DC gear motors with encoders
- Power: 12V LiFePO4 battery pack
- Sensors: Ultrasonic, IMU, GPS, Load cell (optional)
- Communication: Serial and I2C interfaces
- Chassis: Plywood + 3D printed mounts

Getting Started
1. Prerequisites
- Arduino IDE (v2.0+)
- Libraries: ENES100.h (Arduino Library created for this class)

2. Uploading Code
- Clone this repository:
  git clone (https://github.com/Mbaugh447/OTV-Code-Material-Mermaids/)
- Open main.ino in Arduino IDE.
- Select the correct board and port.
- Upload the program.

Team Structure
- Programming Team: Embedded code, drive logic, and sensor integration
- Electronics Team: Circuit design, power distribution, and wiring
- Materials Team: Chassis fabrication, component mounting, and analysis
- Testing & Operations: Field testing, mission performance, and data logging

License
This project is for academic use under the ENES100 course at the University of Maryland. All hardware and code designs are open for educational purposes.

Team Name: Material Mermaids
Course: ENES100, University of Maryland
Semester: Fall 2025
