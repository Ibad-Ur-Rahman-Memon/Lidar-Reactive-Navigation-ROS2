# 🚗 Design and Implementation of a Lidar-Based Reactive Navigation System in Gazebo

![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble-blue)
![Language](https://img.shields.io/badge/Language-Python%203-yellow)
![Simulator](https://img.shields.io/badge/Simulator-Gazebo-orange)
![University](https://img.shields.io/badge/Sukkur%20IBA-University-green)

**Course:** TEL-413 – Robotics & Automation  
**Instructor:** Dr. Ghulam Abbas Lashari  
**Semester:** 7th  
**Department:** Computer Systems Engineering  
**University:** Sukkur IBA University  

---

## 📌 Abstract
This project focuses on the **design and implementation of an autonomous obstacle avoidance system** using **ROS 2 Humble** and **Gazebo**. A **TurtleBot3 Burger** robot is deployed in a custom hexagonal simulation environment.

The system is powered by a custom ROS 2 Python node named `obstacle_avoidance`, which processes real-time data from a **360° LDS-01 LiDAR sensor**. A **reactive navigation algorithm** divides the LiDAR scan into **Front, Left, and Right sectors** and dynamically generates velocity commands to safely avoid obstacles without any prior map information.

This project demonstrates fundamental concepts of **robot perception, decision-making, and motion control** using a reactive control paradigm.

---

## 👥 Team Members
| Name | Student ID |
|-----|------------|
| **Ibad Ur Rahman** (Team Lead) | 133-22-0004 |
| Hamid Ali | 133-22-0023 |
| Muhammad Naeem | 133-22-0028 |
| Syed Hashir Ali | 133-22-0037 |
| Khalid Hussain | 133-22-0011 |

---

## ⚙️ System Architecture
The system follows a classical **Sense → Think → Act** robotics pipeline using standard ROS 2 communication topics:

- **Sensing:** `/scan` (LaserScan data from LiDAR)
- **Decision Making:** Reactive obstacle avoidance logic
- **Actuation:** `/cmd_vel` (Velocity commands)

---

## 🧠 Obstacle Avoidance Logic (Finite State Machine)

The LiDAR scan is segmented into **three regions**:

- **Front**
- **Left**
- **Right**

A safety threshold of **0.5 meters** is used to detect obstacles.

### Navigation States:

1. **🔴 Emergency Stop & Turn (Highest Priority)**
   - **Condition:** Obstacle detected in Front < 0.5 m  
   - **Action:**  
     - Linear Velocity: `v_x = 0.0 m/s`  
     - Angular Velocity: `v_z = -0.6 rad/s` (sharp right turn)

2. **🟡 Avoid Left Obstacle**
   - **Condition:** Obstacle detected on Left < 0.5 m  
   - **Action:**  
     - Linear Velocity: `v_x = 0.15 m/s`  
     - Angular Velocity: `v_z = -0.4 rad/s`

3. **🟡 Avoid Right Obstacle**
   - **Condition:** Obstacle detected on Right < 0.5 m  
   - **Action:**  
     - Linear Velocity: `v_x = 0.15 m/s`  
     - Angular Velocity: `v_z = 0.4 rad/s`

4. **🟢 Cruise Mode (Default State)**
   - **Condition:** No nearby obstacles  
   - **Action:**  
     - Linear Velocity: `v_x = 0.22 m/s`  
     - Angular Velocity: `v_z = 0.0 rad/s`

---

## 🚀 Installation & Execution

### ✅ Prerequisites
- Ubuntu 22.04 LTS
- ROS 2 Humble Hawksbill
- Colcon Build System
- TurtleBot3 ROS 2 Packages
- Gazebo Simulator

---

### 📥 1. Clone the Repository
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/Ibad-Ur-Rahman-Memon/Lidar-Reactive-Navigation-ROS2.git
```

### 🛠️ 2. Build the Workspace
After cloning the repository, build the ROS 2 workspace using **colcon**.

```bash
cd ~/ros2_ws
colcon build
```

### 🌍 3. Launch TurtleBot3 Gazebo Simulation
Start the Gazebo simulation environment and spawn the **TurtleBot3 Burger** robot.

First, set the TurtleBot3 model:
```bash
export TURTLEBOT3_MODEL=burger
```

### ▶️ 4. Run the Obstacle Avoidance Node
After successfully launching the Gazebo simulation, open a **new terminal window** to run the obstacle avoidance node.

First, source the workspace:
```bash
cd ~/ros2_ws
source install/setup.bash
```
## 📁 Project File Structure
The ROS 2 workspace follows the standard **colcon** directory layout. The structure of the project is shown below:

```text
ros2_ws/
├── src/
│   └── <your_package_name>/
│       ├── package.xml
│       ├── CMakeLists.txt
│       ├── src/
│       │   └── <node_file>.py / <node_file>.cpp
│       └── launch/
│           └── <launch_file>.py
├── build/
├── install/
└── log/
```
## 🔮 Future Enhancements
The following improvements can be implemented to extend the functionality and robustness of the system:

- 🗺️ **SLAM Integration**  
  Integrate SLAM algorithms such as **GMapping** or **Cartographer** to enable real-time map generation and localization.

- 🧭 **Nav2 Stack Integration**  
  Incorporate the ROS 2 **Navigation2 (Nav2)** stack to support global path planning and goal-based navigation.

- 🎯 **PID-Based Motion Control**  
  Apply PID controllers for smoother velocity transitions and improved motion stability.

- 🚧 **Dynamic Obstacle Handling**  
  Enhance the system to detect and respond to moving obstacles in real-time.

- 🧪 **Complex Environment Testing**  
  Evaluate the algorithm in more cluttered and dynamic Gazebo environments.

  ## 📜 License & Copyright
**© 2025 Ibad Ur Rahman & Team**  
Department of Computer Systems Engineering, Sukkur IBA University.  

All rights reserved. This project is developed strictly for **academic and educational purposes**.  
Unauthorized use, modification, or distribution without proper permission is prohibited.
