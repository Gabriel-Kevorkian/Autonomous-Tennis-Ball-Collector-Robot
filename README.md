# 🎾 Autonomous Tennis Ball Collector Robot

An intelligent rover that autonomously detects, collects, and delivers tennis balls to a designated collection area using computer vision and embedded systems.

![Project Status](https://img.shields.io/badge/status-completed-success)
![Platform](https://img.shields.io/badge/platform-Raspberry%20Pi%205-red)
![Microcontroller](https://img.shields.io/badge/MCU-PIC18F4550-blue)

## 📋 Table of Contents
- [Overview](#overview)
- [Features](#features)
- [Hardware Requirements](#hardware-requirements)
- [System Architecture](#system-architecture)
- [Installation](#installation)
- [Usage](#usage)
- [Project Structure](#project-structure)
- [How It Works](#how-it-works)
- [Challenges & Solutions](#challenges--solutions)
- [Team](#team)
- [Acknowledgments](#acknowledgments)

## 🎯 Overview

This project demonstrates an autonomous robotic system capable of:
- Detecting yellow tennis balls using computer vision (HSV color space)
- Navigating autonomously toward detected balls
- Collecting up to 3 balls using a mechanical scoop system
- Returning to a designated red-bordered collection area
- Operating in a 4-tile arena with obstacle avoidance

**Project Context**: Final project for COE 521 Embedded Systems course at Lebanese American University.

## ✨ Features

### Computer Vision
- Real-time tennis ball detection using OpenCV and HSV filtering
- Red boundary line detection for autonomous return-to-base
- Temporal filtering for stable detection
- Blind spot handling with IR sensor fusion

### Autonomous Navigation
- Center-tracking algorithm for precise ball alignment
- Search mode with intelligent rotation patterns
- Post-catch behavior (backup and search for next ball)
- Multi-threaded processing for smooth operation

### Smart Collection System
- IR sensor triggers when ball enters blind spot
- Motorized scoop mechanism with gear motors
- Capacity detection (upper IR sensor detects full storage)
- Sequential catch commands for reliable collection

### Multi-Ball Mission
- Tracks collected balls (0/3, 1/3, 2/3, 3/3)
- Automatic mission phase transitions
- Return-to-base mode after all balls collected
- Mission complete notification and stop

## 🛠 Hardware Requirements

### Electronics
- **Raspberry Pi 5** (computer vision & control logic)
- **PIC18F4550** microcontroller (motor control)
- **Pi Camera Module** (90° mounted)
- **2x IR Proximity Sensors** (GPIO 23 & 24)
  - Lower sensor: Ball detection in blind spot
  - Upper sensor: Storage capacity detection
- **L298N Motor Driver** (or equivalent 4-channel controller)
- **4x DC Motors** (rover drive system)
- **2x Gear Motors** (scoop mechanism for increased torque)
- **UART Serial Connection** (Pi ↔ PIC communication)

### Mechanical Components
- Rover 5 chassis or equivalent 4-wheel platform
- Custom wooden scoop mount
- Plastic ball container with elastic retention wires
- Rubber wheels for traction
- Cardboard stabilizer for scoop mechanism

### Power Supply
- 7.4V battery pack for motors
- 5V regulated power for Raspberry Pi
- Separate power for PIC18F4550

## 🏗 System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                     Raspberry Pi 5                          │
│  ┌────────────────────────────────────────────────────┐     │
│  │  Camera (OpenCV)                                   │     │
│  │  • Yellow ball detection (HSV)                     │     │
│  │  • Red line detection                              │     │
│  │  • Object tracking & positioning                   │     │
│  └────────────────────────────────────────────────────┘     │
│                          ↓                                  │
│  ┌────────────────────────────────────────────────────┐     │
│  │  Control Logic                                     │     │
│  │  • State machine (tracking/searching/catching)     │     │
│  │  • IR sensor fusion                                │     │
│  │  • Command generation (W/A/S/D/C/X)                │     │
│  └────────────────────────────────────────────────────┘     │
│                          ↓                                  │
│                    UART (9600 baud)                         │
└─────────────────────────────────────────────────────────────┘
                           ↓
┌─────────────────────────────────────────────────────────────┐
│                     PIC18F4550                              │
│  ┌────────────────────────────────────────────────────┐     │
│  │  UART Interrupt Handler                            │     │
│  │  • Receives commands from Pi                       │     │
│  │  • Decodes movement instructions                   │     │
│  └────────────────────────────────────────────────────┘     │
│                          ↓                                  │
│  ┌────────────────────────────────────────────────────┐     │
│  │  Motor Control                                     │     │
│  │  • 4x drive motors (forward/back/turn)             │     │
│  │  • 2x scoop motors (catch mechanism)               │     │
│  └────────────────────────────────────────────────────┘     │
└─────────────────────────────────────────────────────────────┘
```

## 📦 Installation

### Raspberry Pi Setup

1. **Install Dependencies**
```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install Python packages
pip3 install opencv-python numpy picamera2 pyserial lgpio

# Enable serial port
sudo raspi-config
# Navigate to: Interface Options → Serial Port
# Disable login shell, Enable serial hardware
```

2. **Configure Serial Port**
```bash
# Edit /boot/config.txt
sudo nano /boot/config.txt

# Add these lines:
enable_uart=1
dtoverlay=disable-bt

# Reboot
sudo reboot
```

3. **Clone Repository**
```bash
git clone https://github.com/yourusername/tennis-ball-collector.git
cd tennis-ball-collector
```

### PIC18F4550 Setup

1. **Install MPLAB X IDE** from Microchip's website
2. **Open** `pic_firmware/main.c` in MPLAB X
3. **Configure** XC8 compiler settings
4. **Program** the PIC18F4550 with a PICkit or equivalent programmer

### Hardware Connections

**PIC18F4550 Pin Assignments:**
- `RD4-RD7`: Motor PWM outputs (PW1-PW4)
- `RB0-RB3`: Motor direction pins (D1-D4)
- `RC0-RC1`: Scoop motor control
- `RC6`: UART TX (to Pi RX)
- `RC7`: UART RX (from Pi TX)
- `RD1`: Status LED

**Raspberry Pi GPIO:**
- GPIO 23: Lower IR sensor (ball detection)
- GPIO 24: Upper IR sensor (storage full)
- TX/RX: Serial communication to PIC

## 🚀 Usage

### Running the Rover

```bash
cd raspberry_pi
python3 tennis_ball_rover.py
```

### Configuration Options

Edit the constants in `tennis_ball_rover.py`:

```python
ENABLE_DISPLAY = True          # Show camera feed window
CAMERA_ROTATION = 270          # Camera orientation (0/90/180/270)
TARGET_BALLS = 3               # Number of balls to collect
INITIAL_FORWARD_TIME = 4.0     # Startup forward movement (seconds)
IR_ENABLED = True              # Enable IR sensors
```

### Keyboard Controls (Manual Mode)

Press `m` to toggle manual control mode:
- `W` - Move forward
- `S` - Move backward
- `A` - Turn left
- `D` - Turn right
- `X` - Emergency stop
- `C` - Trigger catch mechanism
- `R` - Reset ball counter
- `Q` - Quit program

### Color Calibration

Press `c` to capture HSV values at the center of the camera frame. Use these to adjust color detection ranges:

```python
# Yellow tennis ball (adjust these values)
self.lower_yellow = np.array([20, 100, 100])
self.upper_yellow = np.array([40, 255, 255])

# Red boundary line
self.lower_red1 = np.array([0, 100, 100])
self.upper_red1 = np.array([10, 255, 255])
```

## 📁 Project Structure

```
tennis-ball-collector/
├── README.md
├── docs/
│   ├── project_requirements.pdf
│   ├── project_report.pdf
│   └── images/
│       ├── rover_front.jpg
│       ├── rover_mechanism.jpg
│       └── circuit_diagram.jpg
├── raspberry_pi/
│   ├── tennis_ball_rover.py      # Main Python code
│   └── requirements.txt           # Python dependencies
├── pic_firmware/
│   ├── main.c                     # PIC18F4550 code
│   ├── conbits.h                  # Configuration bits
│   └── Makefile
└── schematics/
    ├── circuit_schematic.png
    └── wiring_diagram.png
```

## 🔍 How It Works

### State Machine

The rover operates through several states:

1. **Initial Forward Movement** (4 seconds)
   - Moves forward into arena at startup
   
2. **Ball Tracking Mode**
   - Camera detects yellow ball
   - Centers ball horizontally (turn left/right)
   - Approaches ball (move forward)
   
3. **Blind Spot Handling**
   - Ball enters camera blind spot (~20-30cm)
   - Lower IR sensor activates
   - Triggers catch sequence
   
4. **Catch Sequence**
   - Sends 'C' command to PIC
   - Scoop motors lower container
   - Ball captured, scoop raises
   - Increments ball counter
   
5. **Post-Catch Behavior**
   - Backs up for 5 seconds
   - Rotates right until new ball detected
   - Returns to tracking mode
   
6. **Return to Base** (after 3 balls or storage full)
   - Searches for red boundary line
   - Approaches red line
   - Final forward movement (5 seconds)
   - Mission complete - stops

### IR Sensor Logic

**Lower IR (GPIO 23)**: Ball Detection
- Only active during search mode
- Detects ball in camera blind spot
- Triggers catch sequence

**Upper IR (GPIO 24)**: Storage Capacity
- Monitors container fill level
- 0 = Full (triggers return-to-base)
- 1 = Space available

### Communication Protocol

Commands sent from Pi to PIC via UART (9600 baud):
- `W` - Forward
- `S` - Backward  
- `A` - Turn left
- `D` - Turn right
- `C` - Catch ball (lower/raise scoop)
- `X` - Stop all motors

## 🧩 Challenges & Solutions

| Challenge | Solution |
|-----------|----------|
| **Camera blind spot** | Implemented IR sensor fusion for detection when ball is too close |
| **Ball centering accuracy** | Added cardboard guides on sides of scoop for passive alignment |
| **Camera placement** | Mounted at 90° on static part of scoop mechanism |
| **Post-catch behavior** | State machine with backup and rotation until next ball found |
| **Storage detection** | Upper IR sensor detects when 3 balls collected |
| **Black ball limitation** | IR cannot detect black objects - would require camera-only approach |

## 👥 Team

**Group #4** - Lebanese American University

- **Kevin El Murr** (202205341)
- **Charbel Kaddoum** (202207194)  
- **Gabriel Kevorkian** (202207351)

**Instructor**: Dr. Zahi Nakad

**Course**: COE 521 - Embedded Systems

## 🙏 Acknowledgments

- **OpenCV Community** for computer vision resources
- **Raspberry Pi Foundation** for excellent documentation
- **Microchip Technology** for PIC18F4550 datasheets
- **AntBot Project** for robotics inspiration
- **Dagu Robotics** for motor controller reference

## 📝 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 📚 References

1. PIC18F2455/2550/4455/4550 Data Sheet, Microchip Technology Inc., 2009
2. Dagu Robotics, 4 Channel DC Motor Controller Manual, RoboSavvy, 2012
3. OpenCV Tennis Balls Recognition Tutorial, Elphel Wiki
4. GitHub - tennis_ball_detection, aditirao7

---

**⭐ If you found this project helpful, please give it a star!**

**🐛 Found a bug? [Open an issue](https://github.com/yourusername/tennis-ball-collector/issues)**
