# Anafi Drone Control System

This project presents a comprehensive drone control system for the Anafi drone, developed using ROS2. The system encompasses manual control via keyboard inputs, Model Predictive Control (MPC) for navigating to reference points and tracking predefined trajectories, and vision-based 3D position estimation of a target drone utilizing YOLOv8. By integrating MPC with real-time visual detection, the drone can effectively track and follow a moving target drone.

## Table of Contents

- [Features](#features)
- [Installation](#installation)
- [Usage](#usage)
- [Project Structure](#project-structure)
- [Examples](#examples)
- [Dependencies](#dependencies)
- [Contributing](#contributing)
- [License](#license)
- [Contact](#contact)

---

## Features

✅ Keyboard manual control of Anafi drone 

✅ Test process for drone's dynamic state

✅ linear MPC for moving to reference point

✅ Newton-Euler MPC for tracking reference trajectory

✅ 3D pose estimation for target drone by YOLOv8

✅ Tracking the target drone using MPC and 3D pose estimation.

---

## Installation

### Prerequisites

- Ubuntu 22.04  
- ROS2 Humble  
- Python 3.10+  
- Docker (optional for isolation)  
- Parrot Anafi SDK  
- OpenCV, NumPy, SciPy, CasADi, cv_bridge, sensor_msgs, geometry_msgs, etc.

### Clone Repository

```bash
git clone https://github.com/zjiang11/anafi_ws.git
```
---

## Usage

### Build Program

```bash
cd ~/anafi_ws
colcon build
source install/setup.bash
```

### Manual Control

### Test Anafi Drone System state

#### Linear MPC

#### Newton-Euler MPC

### Move to Reference Point

### Track the Reference Trajectory

### Collect Parrot Drone's Figures

### Collect Parrot Drone's Keypoints

### Train YOLO 2D BBox

### Train YOLO 3D BBox

### Persue the Target Drone


