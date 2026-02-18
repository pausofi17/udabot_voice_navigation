# UDABot: Voice-Guided Social Robot Navigation 🤖🎤

[![ROS Version](https://img.shields.io/badge/ROS-Noetic-blue)](http://wiki.ros.org/noetic)
[![Paper](https://img.shields.io/badge/IEEE-Paper_Available-b31b1b)](https://ieeexplore.ieee.org/document/11304447)
[![License](https://img.shields.io/badge/License-MIT-green)](LICENSE)

> **Published in:** [2025 IEEE Ninth Ecuador Technical Chapters Meeting (ETCM)](https://ieeexplore.ieee.org/document/11304447)
> **DOI:** 10.1109/ETCM62451.2025.11304447

## 📸 Demo & Architecture

### 🎥 Live Demo: Voice Interaction & Tracking
[![Watch UDABot in Action](https://img.youtube.com/vi/iQ0sD7DAGe8/hqdefault.jpg)](https://youtube.com/shorts/iQ0sD7DAGe8?feature=share)
*> Click the image above to watch the robot tracking a speaker in real-time.*


## 📖 Overview
UDABot is a social robot designed to locate and navigate towards a speaker using solely sound source localization (SSL). Unlike traditional vision-based interaction, UDABot utilizes a **ReSpeaker 4-Mic Array** to estimate the Direction of Arrival (DOA) and autonomously align its heading with the active speaker.

The system features a **distributed architecture** (Raspberry Pi 4 + PC) and includes **custom firmware modifications** on the embedded controller (OpenCR/ARM Cortex-M7) to achieve hardware-level safety constraints.

## 🚀 Key Features
* **Voice Activity Detection (VAD):** Filters ambient noise to process only human speech.
* **DOA Estimation:** Implements GCC-PHAT (Generalized Cross-Correlation with Phase Transform) for accurate angle detection (< 5° error).
* **PID Control Loop:** Smooths the angular velocity for natural turning behavior.
* **Custom Embedded Drivers:** Rewritten C++ HAL for digital sensor integration (see below).

## ⚡ Embedded Firmware (The "Deep Dive")
To integrate **Sharp GP2Y0D810Z0F Digital Distance Sensors** for low-latency cliff detection, the standard TurtleBot3 firmware was modified, as the default libraries only supported analog inputs.

* **Location:** `firmware/opencr_modified/`
* **Modifications:**
    * **Direct GPIO Access:** Replaced the proprietary `ollo` library with a custom C++ driver accessing **GPIO 15 & 16**.
    * **Hardware-Level Safety:** Implemented a logic gate loop directly on the microcontroller. If `(Sensor_L || Sensor_R)` detects a cliff, the firmware triggers an emergency stop *before* the ROS navigation stack receives the message, reducing reaction latency to milliseconds.

## ⚙️ System Architecture
The project is structured as a ROS Noetic package with the following node graph:

1.  **`audio_processor` (Python):** Interfaces with ALSA drivers, computes DOA, and publishes `/sound_direction`.
2.  **`motion_controller` (Python):** Subscribes to the angle, calculates error, and applies PID correction to publish `/cmd_vel`.
3.  **`firmware_loop` (C++):** Runs on OpenCR, handling odometry and safety interrupts.

## 📂 Project Structure
```
udabot_voice_navigation/
├── firmware/           # Modified C++ OpenCR firmware & drivers
├── launch/             # System-wide launch files
├── scripts/            # ROS Nodes (VAD/DOA and PID Control)
├── docs/               # System diagrams and datasheets
├── CMakeLists.txt      # Catkin build configuration
└── package.xml         # Package dependencies
```
## 🛠️ Installation & Usage
Prerequisites:

* Ubuntu 20.04 (Focal)
* ROS Noetic
* Python dependencies: pyusb, numpy, webrtcvad

## Build
```
    cd ~/catkin_ws/src
    git clone [https://github.com/YOUR_USERNAME/udabot_voice_navigation.git](https://github.com/YOUR_USERNAME/udabot_voice_navigation.git)
    cd ..
    catkin_make
    source devel/setup.bash
```
## Run System

We provide a unified launch file that starts the hardware drivers and the logic nodes:
```
    roslaunch udabot_voice_navigation system.launch
```
## 📚 Citation
If you use this work, please cite our IEEE ETCM 2025 paper:

```bibtex
@inproceedings{contreras2025voice,
  title={Voice-Guided User Localization in a Social Robot},
  author={Contreras, Paula and Iturralde, Daniel and Mejía, David and Cabrera, Andrés and Burbano, Jaime and Fajardo, José and Mora, Esteban and Pintado, Pablo and Duque, Gabriela},
  booktitle={2025 IEEE Ninth Ecuador Technical Chapters Meeting (ETCM)},
  pages={1--6},
  year={2025},
  organization={IEEE},
  doi={10.1109/ETCM62451.2025.11304447}
}


