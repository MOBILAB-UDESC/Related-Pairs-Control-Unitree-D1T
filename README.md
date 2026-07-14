# Related Pairs Control (RPC) — Unitree D1-T 2D Teleoperation

<p align="center">
  <img src="https://raw.githubusercontent.com/MOBILAB-UDESC/Related-Pairs-Control-Unitree-D1T/main/doc/resources/thumb.jpeg" alt="unitree_d1" width="1800"/>
</p>

## Description
A real-time, low-cost 3D teleoperation framework for the **Unitree Robotics D1-T** (6-DOF anthropomorphic manipulator) using only a conventional 2D RGB webcam. 

This project introduces the **Related Pairs Control (RPC)** method, which associates specific pairs of human body keypoints with individual robotic actuators. By combining Google MediaPipe's **Pose Landmarker** and **Hand Landmarker**, the system eliminates the need for expensive depth sensors while maintaining precise control over both the arm and gripper.

**Note:** This package is built on top of the [MOBILAB-UDESC arms](https://github.com/MOBILAB-UDESC/arms.git) ROS 2 meta-package.

## ROS 2 Info
| Ubuntu | ROS 2 Distro | Gazebo Version | Python Dependencies |
| :---: | :---: | :---: | :---: |
| [24.04](https://ubuntu.com/blog/tag/ubuntu-24-04-lts) | [Jazzy](https://docs.ros.org/en/jazzy/index.html) | [Harmonic](https://gazebosim.org/docs/harmonic/getstarted/) | `opencv-python`, `mediapipe` |

---

## Cloning and Building

```cli
mkdir -p ~/arms_ws/src && cd ~/arms_ws/src
git clone [https://github.com/MOBILAB-UDESC/arms.git](https://github.com/MOBILAB-UDESC/arms.git) .
git submodule update --init unitree_d1 d1_2f

# Clone the RPC Vision Teleoperation package
git clone [https://github.com/rngllz/Related-Pairs-Control-D1T.git](https://github.com/rngllz/Related-Pairs-Control-D1T.git)

cd ..
rosdep install --from-paths src --ignore-src -r -y
pip install mediapipe opencv-python
colcon build
source install/setup.bash
```

---

## Quick Start Tutorial

### 1. Launch the Robot
You can run the teleoperation system using either the Gazebo simulation or the real hardware.

**Option A: Gazebo Simulation**
```cli
ros2 launch arms_bringup arm_gazebo_launch.py use_sim_time:=true arm:=unitree_d1 gripper:=d1_2f
```

**Option B: Real Hardware (Physical D1-T)**
```cli
ros2 launch arms_bringup arm_bringup.launch.py arm_profile:=unitree_d1_2f use_sim_time:=false
```

### 2. Run the Vision Controller
Open a new terminal tab, source the workspace, and start the MediaPipe vision node:
```cli
cd ~/arms_ws
source install/setup.bash
python3 src/Related-Pairs-Control-D1T/control_d1t.py
```

---

## Keyboard Controls
The system relies on a keyboard interface for real-time mode switching and safety:

| Key | Action | Function |
| :---: | :---: | :--- |
| **`t`** | **Activate** | Starts user range-of-motion calibration |
| **`u`** | **Deactivate** | Saves calibration limits and normalizes workspace |
| **`e`** | **Toggle Mode** | Switches between **Arm Mode** (3D spatial) and **Gripper Mode** (hand gestures) |
| **`o`** | **Toggle ROS 2** | Enables or disables sending commands to the robot (**ON / OFF**) |

---

## How It Works
* **Arm Mode:** Tracks the user's right shoulder and elbow via *Pose Landmarker*, combined with the wrist and middle finger from *Hand Landmarker*, mapping 2D planar movements to 3D joint angles using anatomical visual pairings.
* **Gripper Mode:** Uses high-precision hand tracking to control gripper opening/closing (thumb-to-index distance) and wrist rotation (palm tilt triangulation).
* **Safety First:** Data is only published to `/joint_trajectory` or `/gripper_cmd` when ROS 2 transmission is toggled **ON** via the `o` key.

---

## Demonstrations

<p align="center">
  <img src="https://raw.githubusercontent.com/MOBILAB-UDESC/Related-Pairs-Control-Unitree-D1T/main/doc/resources/arm.png" alt="MediaPipe Tracking" width="400"/>
  <img src="https://raw.githubusercontent.com/MOBILAB-UDESC/Related-Pairs-Control-Unitree-D1T/main/doc/resources/gripper.png"alt="Gazebo Simulation" width="400"/>
</p>

---

## Video Demonstration

A full demonstration of the **Related Pairs Control (RPC)** system in action—showing real-time 3D camera tracking synchronizing with the Unitree D1-T—is available on YouTube.

[![Watch the video on YouTube](https://img.shields.io/badge/YouTube-Watch%20Video-red?style=for-the-badge&logo=youtube&logoColor=white)](https://www.youtube.com/watch?v=5eqauxCYT18)

---



## Reference and Citation
Accepted for presentation and publication in the Proceedings of the **XXVI Brazilian Congress of Automatica (CBA 2026)**, São Paulo, Brazil.

> **Title:** "Related Pairs Control (RPC): A 3D Position Controller for an Anthropomorphic Arm Using Two-Dimensional Images"
