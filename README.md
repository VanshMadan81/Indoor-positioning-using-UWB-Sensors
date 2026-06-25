# Indoor-positioning-using-UWB-Sensors
# 🚁 UWB-Based Drone Localization Using Extended Kalman Filter in ROS 2

[![ROS 2 Humble](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/)
[![Gazebo Fortress](https://img.shields.io/badge/Gazebo-Fortress-orange.svg)](https://gazebosim.org/)
[![Python](https://img.shields.io/badge/Python-3.10-green.svg)](https://www.python.org/)
[![License](https://img.shields.io/badge/License-MIT-lightgrey.svg)](LICENSE)

## 📌 Overview

This project presents a simulation framework for **Ultra-Wideband (UWB) based drone localization** using **ROS 2 Humble** and **Gazebo Fortress**. The objective is to estimate the real-time position of an aerial vehicle in GPS-denied environments using distance measurements from multiple UWB anchors.

Two localization techniques are implemented and compared:

- **Least Squares (LS) Localization**
- **Extended Kalman Filter (EKF) Localization**

The project simulates UWB ranging, processes noisy distance measurements, estimates the drone position, and visualizes both the estimated trajectory and anchor locations in RViz.

---

## ✨ Features

- UWB anchor-based localization
- ROS 2 Humble compatible
- Gazebo Fortress simulation
- Least Squares position estimation
- Extended Kalman Filter implementation
- RViz visualization
- Configurable anchor placement
- Noise simulation for realistic ranging
- Modular ROS 2 package structure
- Easy integration with real UWB hardware

---

# 🏗️ System Architecture

```
                UWB Anchors
                      │
        Distance Measurements
                      │
             UWB Localizer Node
                      │
        ┌─────────────┴─────────────┐
        │                           │
 Least Squares Estimator      EKF Estimator
        │                           │
        └─────────────┬─────────────┘
                      │
              Estimated Position
                      │
                  RViz Visualization
```

---

# 📂 Project Structure

```
uwb_localization/
│
├── config/
│   ├── anchors.yaml
│
├── launch/
│   ├── localization.launch.py
│
├── rviz/
│   ├── localization.rviz
│
├── src/
│   ├── least_squares_localizer.py
│   ├── ekf_localizer.py
│   ├── uwb_simulator.py
│   ├── rviz_visualizer.py
│
├── package.xml
├── setup.py
└── README.md
```

---

# ⚙️ Localization Pipeline

1. Generate UWB distance measurements.
2. Add configurable Gaussian noise.
3. Receive ranges from multiple anchors.
4. Compute initial position using Least Squares.
5. Refine estimation using Extended Kalman Filter.
6. Publish estimated position.
7. Visualize results in RViz.

---

# 🛰️ UWB Anchor Configuration

The simulation uses six fixed UWB anchors positioned around the operating area.

Example:

| Anchor | X (m) | Y (m) |
|---------|------:|------:|
| A1 | -3 | -3 |
| A2 | 3 | -3 |
| A3 | -3 | 0 |
| A4 | 3 | 0 |
| A5 | -3 | 3 |
| A6 | 3 | 3 |

The anchor configuration can be modified to suit different environments.

---

# 📡 ROS 2 Topics

| Topic | Description |
|--------|-------------|
| `/uwb_distances` | Simulated UWB distance measurements |
| `/ls_position` | Least Squares estimated position |
| `/ekf_position` | EKF estimated position |
| `/imu` | IMU measurements |
| `/odom` | Robot odometry |

---

# 🚀 Running the Simulation

## Clone the repository

```bash
git clone https://github.com/<username>/<repository>.git
```

## Build

```bash
cd ~/ros2_ws

colcon build

source install/setup.bash
```

## Launch

```bash
ros2 launch uwb_localization localization.launch.py
```

---

# 📊 Localization Methods

## Least Squares (LS)

Computes the position estimate directly from multiple UWB distance measurements.

### Advantages

- Fast computation
- Simple implementation
- Good initial estimate

---

## Extended Kalman Filter (EKF)

Fuses previous state estimates with current UWB measurements to improve localization accuracy.

### Advantages

- Reduced noise
- Smooth trajectory
- Higher robustness
- Better real-time performance

---

# 📈 Results

The project compares:

- Ground Truth Position
- Least Squares Estimate
- EKF Estimate

Performance is evaluated using localization error over time.

The EKF consistently produces smoother and more accurate trajectories than the Least Squares estimator.

---

# 🛠️ Technologies Used

- ROS 2 Humble
- Gazebo Fortress
- RViz
- Python 3
- NumPy
- SciPy
- geometry_msgs
- visualization_msgs

---

# 🔮 Future Work

- 3D localization
- Bias-aware EKF
- Anchor Self Localization (ASL)
- Real Pozyx UWB hardware integration
- PX4/ArduPilot drone integration
- Multi-drone localization
- Adaptive noise estimation
- SLAM integration

---

# 📷 Demo

Add screenshots or GIFs here.

```
docs/images/rviz.png
docs/images/gazebo.png
docs/images/localization.gif
```

---

# 🤝 Contributing

Contributions are welcome.

1. Fork the repository.
2. Create a feature branch.
3. Commit your changes.
4. Submit a Pull Request.

---

# 📄 License

This project is released under the MIT License.

---

# 👨‍💻 Author

**Vansh Madan**

Indian Institute of Technology Kanpur (IIT Kanpur)

GitHub: https://github.com/<your-github-username>

---

## ⭐ Acknowledgements

- ROS 2 Community
- Gazebo Sim
- Open Robotics
- Pozyx UWB Documentation
- ArduPilot & PX4 Open Source Communities
