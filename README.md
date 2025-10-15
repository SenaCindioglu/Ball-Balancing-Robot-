# Ball Balancing Robot

A sophisticated self-balancing robotic platform that maintains dynamic equilibrium on top of a spherical surface through integrated computer vision, advanced control algorithms, and real-time firmware execution. This project demonstrates the practical application of control theory, image processing, and embedded systems in managing an inherently unstable inverse pendulum system with omnidirectional motion constraints.

## Overview

This system combines three core technologies to achieve autonomous ball balancing:
- **Computer Vision**: Real-time ball and robot position detection using OpenCV
- **PID Control**: Advanced feedback control for stability management
- **STM32 Firmware**: Embedded motor control and command execution
- **GUI Dashboard**: Intuitive waypoint planning and system monitoring

## Computer Vision Module

The vision system captures real-time camera frames to extract critical spatial information about the robot's state. Using OpenCV-based image processing, the module detects the white ball's position within the frame and identifies the robot's orientation markers.

### Key Features:
- **Color-space conversion** for robust ball detection across lighting conditions
- **Contour detection** to precisely locate the ball center and calculate pixel coordinates
- **Noise filtering** to reduce lighting variations and motion blur artifacts
- **Real-time processing** at high frame rates for responsive corrections
- **Position output**: Absolute (x, y) coordinates and robot angular orientation

### Data Flow:
Camera Input → Color Space Conversion → Edge Detection → Contour Analysis → Ball Center Calculation → Position Feedback

## PID Controller Implementation

The PID (Proportional-Integral-Derivative) control algorithm forms the core of the stability system, processing vision feedback to generate motor commands.

### Control Components:

**Proportional (P)**: Generates immediate corrective action proportional to current error magnitude. Larger errors produce stronger corrections but can cause oscillation if overly aggressive.

**Integral (I)**: Accumulates past deviations to eliminate steady-state error. Prevents the system from settling at offset positions and ensures the ball reaches true equilibrium.

**Derivative (D)**: Predicts future error trends to prevent oscillation and overshoot. Dampens rapid corrections and stabilizes the system response.

### Tuning Parameters:
- `Kp` (Proportional Gain): Affects response speed and stability
- `Ki` (Integral Gain): Corrects long-term drift and steady-state error
- `Kd` (Derivative Gain): Reduces overshoot and oscillation

### Control Loop:
The algorithm runs in a tight feedback loop, calculating independent corrections for X and Y axes to manage the robot's pitch and roll. Error signals represent deviation between current ball position and desired equilibrium point (typically frame center). Motor command outputs drive stepper or brushless motors in opposite directions to tilt the platform and shift the ball back toward equilibrium.

## STM32 Microcontroller Firmware

The STM32 microcontroller executes low-level motor control and receives high-level commands from the computer vision system via serial communication (UART).

### Core Responsibilities:

**Motor Control**: Implements PWM (Pulse Width Modulation) drivers for precise motor speed and direction control. Manages motor acceleration curves and current limiting to prevent hardware damage.

**Command Processing**: Receives motor command values from the PID controller and translates them into appropriate PWM signals sent to motor drivers (L298N or similar H-bridge modules).

**Sensor Integration**: Processes IMU data (accelerometer/gyroscope) for hybrid feedback and cross-validation with vision data. Provides redundancy and improved stability.

**Deterministic Timing**: Maintains control loop at fixed intervals (typically 50-100 Hz), ensuring reliable timing for motor pulses and serial data reception.

**Waypoint Management**: Receives target waypoint commands from the GUI via serial protocol. Stores trajectory commands and executes sequential movements for autonomous navigation.

### Serial Protocol:
Host → STM32: Motor commands (x, y motor PWM values), waypoint targets
STM32 → Host: Sensor feedback, status messages, system diagnostics

## GUI Control Dashboard

The graphical interface provides intuitive waypoint definition and real-time system monitoring.

## System Architecture

```
Camera
   ↓
Vision Processing (OpenCV)
   ↓
Ball/Robot Detection
   ↓
Position Coordinates (x, y, θ)
   ↓
PID Controller
   ↓
Motor Commands
   ↓
STM32 UART Interface
   ↓
PWM Drivers
   ↓
Motors
   ↓
Physical Motion
   ↓
Feedback Loop (Back to Vision)
```

https://github.com/user-attachments/assets/d9fc9d44-12c5-4ad0-847a-762982dbec53


https://github.com/user-attachments/assets/8340e065-8bae-4b92-afb5-ee4f54b2047f

<img width="1199" height="841" alt="Screenshot 2025-10-12 155036" src="https://github.com/user-attachments/assets/fb8edf5a-6342-434e-abc9-5355b4d044a4" />
