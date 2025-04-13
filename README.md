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
### Derive the Quaternions for Representing Pose
The following code use /tf topic to subscribe ros2 message of 'TFMessage' for the pursuing drone (Anafi) and target drone (Bebop1).
One way to accquire the TFMessage for both drones is to use Vicon System in Martin Barczyk's lab, (211, Mechenical Engineering Building, University of Alberta). The detail of setting up the Vicon system can be refered to https://gitlab.com/barczyk-mechatronic-systems-lab/anafi_ros2/-/tree/joshua?ref_type=heads

### Manual Control

```bash
ros2 run anafi_test manual_control.py
```
Notice: 

key.right: Take off

key.left: Landing

w: X Direction (Forward)

s: X Direction (Backward)

a: Y Direction (Left)

d: Y Direction (Right)

r: Z Direction (Upward)

f: Z Direction (Downward)

c: Yaw Direction (Anticlockwsie)

x: Yaw Direction (Clockwise)


### Test Anafi Drone System state

#### Linear MPC

Implementing linear state function for MPC on drone

##### Collect Anafi Drone's Data

```bash
ros2 run anafi_test collect_anafi_data_linear_mpc.py
```
notice:

Enter testing axis after using "key.right" to take off the drone.

Using keyboard for manual control (right, left, w, s, a, d, r, f, c, x) can swith from the tracking mode to manual mode for safety.

##### Calculate and test Anafi Drone's System State Matrix

```bash
python3 src/anafi_test/process_data/calculate_state_function_linear_mpc.py
python3 src/anafi_test/process_data/test_state_function_linear_mpc.py
```

#### Newton-Euler MPC

Implementing reference roll, pitch angles for MPC

##### Collect Anafi Drone's Data

```bash
ros2 run anafi_test collect_anafi_data_newton_euler_mpc.py
```
notice:

Enter testing axis after using "key.right" to take off the drone.

Using keyboard for manual control (right, left, w, s, a, d, r, f, c, x) can swith from the tracking mode to manual mode for safety.

##### Calculate and test Anafi Drone's System State Matrix

```bash
python3 src/anafi_test/process_data/calculate_state_function_newton_euler_mpc.py
python3 src/anafi_test/process_data/test_state_function_newton_euler_mpc.py
```

### Move to Reference Point by Linear MPC

```bash
ros2 run anafi_test move2point_linear_mpc.py
```
notice:

input four float value: x,y,z,yaw for reference point

Using keyboard for manual control (right, left, w, s, a, d, r, f, c, x) can swith from the MPC mode to manual mode for safety.

### Track the Reference Trajectory

```bash
ros2 run anafi_test move2point_linear_mpc.py
```
notice:

input eight float value: x,y,z,yaw for initial reference point. x_speed, y_speed, z_speed, yaw_speed for reference point moving speed.

The trajectory is in the shape of the circle. Larger x_speed, y_speed will enhance radius of the circle. The x_speed, y_speed, z_speed and yaw_speed should not exceed 0.3 at the first attempt. User can adjust the trajectory by revising "def update_ref_state_callback" in move2point_linear_mpc.py

Using keyboard for manual control (right, left, w, s, a, d, r, f, c, x) can swith from the MPC mode to manual mode for safety.

### Plot the Drone's Trajectory

```bash
python3 src/anafi_test/process_data/plot_move2point_linear_mpc.py
python3 src/anafi_test/process_data/plot_move2point_newton_euler_mpc.py
```

### Collect Parrot Drone's Figures

```bash
ros2 run track_parrot collect_parrot_fig.py
```
### Label 2D Images for YOLO 2D BBox Model Training

```bash
pip install labelImg
labelImg
```

### Collect Parrot Drone's Keypoints for YOLO 3D BBox Model Training

```bash
ros2 launch get_keypoint_launch.py
```

### Train and Test YOLO 2D BBox

```bash
python3 src/track_parrot/train_drone_yolo_2d/train.py
python3 src/track_parrot/train_drone_yolo_2d/test.py
```

### Train YOLO 3D BBox

```bash
python3 src/track_parrot/train_drone_yolo_3d/train.py
python3 src/track_parrot/train_drone_yolo_3d/test.py
```

### Persue the Target Drone

```bash
ros2 launch pursuer_launch_yolo.py
```
notice:

Click 'Start Tracking' after using "key.right" to take off the drone.

Using keyboard for manual control (right, left, w, s, a, d, r, f, c, x) can swith from the tracking mode to manual mode for safety.

### PLot Tracking Process

```bash
python3 src/track_parrot/process_data/plot_tracking_data.py
```
 
