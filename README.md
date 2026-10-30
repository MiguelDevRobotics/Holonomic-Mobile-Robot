# Economical Omnidirectional Mobile Robot (ROS 1 Noetic)

An open-source, cost-effective omnidirectional mobile robot designed for testing and development. This repository integrates mechanical design, custom electronics (PCB), and ROS 1 software architecture.

---

## System Overview & Tech Stack

* **ROS Version:** ROS 1 Noetic
* **Firmware:** Arduino IDE (`rosserial` over serial/USB)
* **Hardware/Electronics:** Custom PCB designed in Fusion 360 to control motors with encoders.
* **Mechanical Design:** Full CAD assembly and models available in Fusion 360.
* **Sensors:** LiDAR STL (`ldlidar_stl_ros` included as a submodule).


---

##  Cloning and Installation

Because this repository contains a **Git submodule** for the LiDAR driver, clone it recursively:

```bash
git clone --recursive https://github.com/MiguelDevRobotics/Holonomic-Mobile-Robot.git
```

If you already cloned it without submodules, initialize them with:

```bash
git submodule update --init --recursive
```

---

##  Building the ROS 1 Workspace

1. Navigate to the catkin workspace:
   ```bash
   cd robot_omni_ws
   ```

2. Install workspace dependencies:
   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. Build using `catkin_make`:
   ```bash
   catkin_make
   ```

4. Source your workspace:
   ```bash
   source devel/setup.bash
   ```

---

## Firmware (Arduino IDE & rosserial)

1. Open the sketch located in the `odom-twist/` directory using the **Arduino IDE**.
2. Make sure you have the `rosserial_arduino` library installed in your Arduino environment.
3. Upload the sketch to your esp32 microcontroller.
4. Run the rosserial node in ROS 1 to establish communication:
   ```bash
   rosrun rosserial_python serial_node.py /dev/ttyUSB0
   ```

---
