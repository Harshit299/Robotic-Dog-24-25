# Robotic Dog Project

This repository contains code and documentation for a four-legged robotic dog, utilizing inverse kinematics to control leg movements. The project includes Python scripts for 3D simulation and Arduino code for servo motor control, enabling precise positioning of each leg's end effector.

## Table of Contents

- Project Overview
- Repository Structure
- Hardware Requirements
- Software Requirements
- Installation and Setup
- Usage
- Coordinate System and Offsets
- Kinematics and Transformation Matrices
- Angle Ranges
- Photo Gallery

## Project Overview

The robotic dog features four legs (front right, front left, back right, back left), each with three degrees of freedom driven by servo motors. Inverse kinematics calculates joint angles (T1, T2, T3) to position each leg at specified (x, y, z) coordinates. Python scripts provide 3D visualization, while Arduino code controls the physical robot. The robot's front aligns with the XY plane, with fixed offsets for each leg.

## Repository Structure


- `Arduino Files/`

  - `Left-Back-Leg-Arduino.ino`: Firmware for controlling the left back leg servos.
  - `Left-Front-Leg-Arduino.ino`: Firmware for controlling the left front leg servos.
  - `Right-Back-Leg-Arduino.ino`: Firmware for controlling the right back leg servos.
  - `Right-Front-Leg-Arduino.ino`: Firmware for controlling the right front leg servos.

- `Python File/`

  - `kinematics/`
    - `InverseKinematics.py`: Script to compute joint angles for left and right legs.
  - `simulation/`
    - `4 Legs Simulation.py`: 3D simulation of all four legs with interactive controls.
    - `Left Leg Simulation.py`: 3D simulation of a single left leg.
    - `Right Leg Simulation.py`: 3D simulation of a single right leg.

- `Solidworks File/`

  - (Placeholder for SolidWorks design files, e.g., `.sldprt` or `.sldasm` files)

- `docs/`

  - `ReadMe.md`: Main project overview and instructions.
  - `arduino.md`: Detailed documentation for Arduino firmware.
  - `components.md`: Information on design specifications and hardware components.
  - `python.md`: Detailed documentation for Python simulation and kinematics scripts.

## Hardware Requirements

- Arduino-compatible microcontroller (e.g., Arduino Uno)
- 12 servo motors (3 per leg)
- Power supply for servos (5V, sufficient current)
- Robotic dog chassis (leg segments: a1=4.5 cm, a2=6 cm, a3=5.4 cm)
- USB cable
- Jumper wires and prototyping board

## Software Requirements

- **Python 3.x** with:
  - `numpy`
  - `matplotlib`
- **Arduino IDE**
- Serial monitor (e.g., Arduino IDE's Serial Monitor)

## Installation and Setup

1. **Python Setup**:

   - Install Python 3.x.

   - Install libraries:

     ```bash
     pip install numpy matplotlib
     ```

2. **Arduino Setup**:

   - Install Arduino IDE from arduino.cc.
   - Connect servos to Arduino pins (default: 9, 10, 11).
   - Upload the appropriate `.ino` file for each leg.

3. **Simulation**:

   - Run Python scripts for visualization.
   - Adjust joint angles using sliders.

4. **Physical Robot**:

   - Power servos and connect to Arduino.
   - Use Serial Monitor (9600 baud) to input coordinates.

## Usage

### Simulation

- Run `4LegsForwardSimulation.py` for all legs:

  ```bash
  python 4LegsForwardSimulation.py
  ```

  - Adjust T1, T2, T3 via sliders.
  - Use XY, XZ, YZ buttons to change views.
  - View end effector coordinates on the plot.

- Run `LeftLegSimulation.py` or `RightLegSimulation.py` for single-leg simulation:

  ```bash
  python LeftLegSimulation.py
  python RightLegSimulation.py
  ```

### Inverse Kinematics

- Run `InverseKinematicsBothLeg.py`:

  ```bash
  python InverseKinematicsBothLeg.py
  ```

  - Input leg (0 for right, 1 for left) and (x, y, z).
  - Outputs T1, T2, T3 in degrees.

### Arduino Control

- Upload `.ino` file for each leg.
- Open Serial Monitor (9600 baud).
- Enter x, y, z coordinates to move servos.

## Coordinate System and Offsets

The coordinate system is centered at the robot's body, with the front in the XY plane. Offsets (in cm):

- **Right Front**: `x = x_centre - 1`, `y = y_centre`, `z = z_centre - 8.5`
- **Right Back**: `x = x_centre - 1`, `y = y_centre`, `z = z_centre + 8.5`
- **Left Front**: `x = x_centre + 1`, `y = y_centre`, `z = z_centre - 8.5`
- **Left Back**: `x = x_centre + 1`, `y = y_centre`, `z = z_centre + 8.5`

## Kinematics and Transformation Matrices

Leg segments: `a1 = 4.5 cm`, `a2 = 6 cm`, `a3 = 5.4 cm`.

### Rotation Matrices

```python
R0_1 = np.array([[-np.sin(T1), 0, np.cos(T1)],
                 [np.cos(T1),  0, np.sin(T1)],
                 [0,           1,          0]])

R1_2 = np.array([[np.cos(T2), -np.sin(T2), 0],
                 [np.sin(T2),  np.cos(T2), 0],
                 [0,           0,          1]])

R2_3 = np.array([[1, 0, 0],
                 [0, 0, 1],
                 [0, -1, 0]])
```

### Displacement Vectors

```python
D0_1 = np.array([[a1 * np.cos(T1)], [a1 * np.sin(T1)], [0]])
D1_2 = np.array([[a2 * np.cos(T2)], [a2 * np.sin(T2)], [0]])
D2_3 = np.array([[a3 * np.cos(T3)], [a3 * np.sin(T3)], [0]])
```

### Homogeneous Transformation Matrices

```python
H0_1 = np.concatenate((R0_1, D0_1), axis=1)
H0_1 = np.concatenate((H0_1, [[0, 0, 0, 1]]), axis=0)

H1_2 = np.concatenate((R1_2, D1_2), axis=1)
H1_2 = np.concatenate((H1_2, [[0, 0, 0, 1]]), axis=0)

H2_3 = np.concatenate((R2_3, D2_3), axis=1)
H2_3 = np.concatenate((H2_3, [[0, 0, 0, 1]]), axis=0)

H0_2 = np.dot(H0_1, H1_2)
H0_3 = np.dot(H0_2, H2_3)
```

## Angle Ranges

- **Right Leg**: T1: -90° to +90°; T2, T3: 0° to 180°
- **Left Leg**: T1: 90° to 270°; T2, T3: 0° to 180°

## Photo Gallery

- Manipulator Diagram:
  ![Manipulator diagram (2)](https://github.com/user-attachments/assets/547689ec-47d4-4785-8a73-501576a69edf)

- Robotic Dog(Model):
  
  ![0 (1)](https://github.com/user-attachments/assets/e60c13fa-f6de-4905-8958-86fc56c41778)
- Robotic Dog(Actual Bot):
  ![WhatsApp Image 2025-04-16 at 18 10 43_51172742](https://github.com/user-attachments/assets/fe38dabb-26d7-484d-bc63-3f1190e2d42a)
