# Pololu 3pi Line Following Robot

## Overview
This repository contains the firmware for a line-following robot. It includes sensor integration for line tracking, motor control for movement, and a state machine for different phases of navigation.

### Sensor Integration
- Utilises line sensors (`LineSensor.h`) and encoders (`encoders.h`) for detecting the line and measuring wheel rotation.

### Motor Control
- Controls the motors using the `motors.h` module, allowing for speed and direction adjustments.

### Line Following Algorithm
- The `lineFollowing` function dynamically adjusts motor speeds to maintain line tracking, steering the robot by varying power to the wheels.

### Kinematics Calculations
- Functions are included to calculate the distances traveled by the wheels and the robot's orientation relative to its starting position.

### State Machine
The robot's behavior is managed by a state machine within the `loop` function:
- **Mode 1**: Initialization state.
- **Mode 2**: Searching for the line.
- **Mode 3**: Aligning with the line.
- **Mode 4**: Line tracking.
- **Mode 5**: Responding to line loss.
- **Mode 6**: Returning to start.

### Gap Detection
- The program can detect gaps in the line, allowing the robot to handle intersections or breaks in the path.

### End Behavior
- Once the course is complete, indicated by gap flags, the robot ceases movement by setting motor power to zero.

### Return to Start
- The `kinematics` function calculates the return path to the starting position, ensuring the robot can navigate back to its origin.

## Usage
1. Ensure all hardware components are correctly connected.
2. Flash this program onto your robot's microcontroller.
3. Place the robot on the line and power it on.
4. The robot will autonomously follow the line and return to the start when finished.
