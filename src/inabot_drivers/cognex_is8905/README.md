# Cognex IS8905 ROS 2 Node

This package provides a ROS 2 Humble C++ node to interface with the **Cognex IS8905** camera.  
The node connects via Ethernet and publishes camera detection values (0, 1, 2) to a ROS 2 topic.

---

## 📦 Features

- Connects to one or multiple IS8905 cameras via TCP/IP
- Publishes detection results as `std_msgs/msg/Int32`
- Supports multiple cameras via separate node instances or launch files
- Easy integration with other ROS 2 nodes for navigation, perception, or logging

---

## ⚙ Camera Specifications

- **Temperature**: -20°C to 80°C (-4°F to 176°F)  
- **Humidity**: <95% non-condensing  
- **Protection**: IP40 (with all cables and lens properly attached)  
- **Shock (Shipping and Storage)**: IEC 60068-2-27, 80 Gs, half-sinusoidal, with lens attached  
- **Vibration (Shipping and Storage)**: IEC 60068-2-6, 10 Gs, 10–500 Hz, 2 hours per axis  
- **Regulations**: CE, FCC, KCC, EU RoHS, China RoHS  
- **LIN Compatible**: LIN 2.0 / 2.1 / 2.2 / 2.2A  
- **Transmit Data Rate**: up to 20 kbps  

---

## 🔧 Installation

1. Clone the package into your workspace:

colcon build --symlink-install
colcon build --symlink-install --packages-select cognex_is8905
source install/setup.bash
ros2 launch cognex_is8905 cognex_is8905_launch.py