# Hexapod Project — Installation Guide

This guide explains how to install and configure the **Hexapod Project** on:
- **PC (Laptop / Desktop)**
- **Raspberry Pi**
- **Arduino Mega**

---
## Setup Characteristics (Reference System)

- **Laptop:** ASUS TUF Gaming F15 (NVIDIA RTX 3060) — Ubuntu 24.04 LTS  
- **Raspberry Pi:** Raspberry Pi 4 (4 GB RAM) — Ubuntu 24.04 LTS  
- **Smartphone:** Google Pixel 7 (used as camera and navigation sensors)
- **Microcontroller:** Arduino Mega  

---
## IMPORTANT NOTE (YOLO & GPU)

To use **YOLO-based vision** with good performance, a **dedicated GPU** is strongly recommended.

- If you **have a GPU** → real-time object detection is achievable.
- If you **do not have a GPU** → the package will still work, but with **limited performance**.

---
## 1. PC installation
### 1.1. Install ROS2 compatible with your Ubuntu distribution 
- Ubuntu 22.04: [ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
- Ubuntu 24.04: [ROS2 Rolling](https://docs.ros.org/en/rolling/Installation/Ubuntu-Install-Debians.html) or [ROS2 Jazzy (Recommended)](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debians.html)

###  1.2. Install colcon:
```bash
sudo apt install python3-colcon-common-extensions
```
### 1.3. Source ROS2 and colcon permanently:
```bash
echo "source /opt/ros/rolling/setup.bash" >> ~/.bashrc
echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
```
OBS: Change `rolling` with your ROS2 distribution (`humble`, `jazzy`, etc.)
### 1.4. Intall Gazebo
```bash
sudo apt-get install ros-rolling-gazebo-ros-pkgs
```
OBS: Change `rolling` with your ROS2 distribution (`humble`, `iron`, etc.)

### 1.5. Install IriunWebCam
```bash
wget https://iriun.gitlab.io/iriunwebcam-2.9.deb
sudo apt install ./iriunwebcam-2.9.deb
sudo apt --fix-broken install
```

### 1.5. Check GPU support fon CUDA (if you don't have a GPU, skip this step)
```bash
nvidia-smi
```
OBS: If you have a GPU, you should see something like this:

```bash
+-----------------------------------------------------------------------------+
| NVIDIA-SMI 510.79.02    Driver Version: 510.79.02    CUDA Version: 11.6     |
|-------------------------------+----------------------+----------------------+
| GPU  Name        Persistence-M| Bus-Id        Disp.A | Volatile Uncorr. ECC |
...
+-----------------------------------------------------------------------------+
```

If you don't see you can try to intall drivers by ubuntu autodetect system

```bash
sudo apt update
sudo apt install ubuntu-drivers-common
sudo ubuntu-drivers autoinstall
sudo reboot
```

### 1.6. Create and activate a virtual environment for YOLO processing:
```bash
python3 -m venv ~/.venvs/yolo
source ~/.venvs/yolo/bin/activate
```
OBS: To use python environment always open an external terminal and activate the environment separatly of the terminal where you are going to run the main code.

### 1.7. Install YOLO dependencies in the virtual environment:
```bash
pip install torch torchvision torchaudio
pip install ultralytics
pip install opencv-python
```
### 1.8. Install opencv-python in the system:
```bash
sudo apt install python3-opencv
```

### 1.9. Create a ROS2 workspace:
```bash
mkdir -p ~/ros2_hex_tutorial_ws/src
cd ~/ros2_ws/src
```
### 1.10. Clone the repository into src directory:
```bash
git clone https://github.com/carlobeni/hexapod_pkg.git
```
### 1.11. Build the workspace:
```bash
cd ~/ros2_hex_tutorial_ws
colcon build --symlink-install
```
### 1.12. Source the workspace permanently:
```bash
echo "source ~/ros2_hex_tutorial_ws/install/setup.bash" >> ~/.bashrc
```

## 2. Raspberry Pi installation
### 2.1. Install ROS2 compatible with your Ubuntu distribution
- Ubuntu 22.04: [ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
- Ubuntu 24.04: [ROS2 Rolling](https://docs.ros.org/en/rolling/Installation/Ubuntu-Install-Debians.html) or [ROS2 Jazzy (Recommended)](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debians.html)

### 2.2. Install colcon
```bash
sudo apt install python3-colcon-common-extensions
```
### 2.3. Source ROS2 and colcon permanently:
```bash
echo "source /opt/ros/rolling/setup.bash" >> ~/.bashrc
echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
```
OBS: Change `rolling` with your ROS2 distribution (`humble`, `jazzy`, etc.)

### 2.4. Install dependencies
```bash
sudo apt update
sudo apt install python3-smbus
sudo apt install python3-smbus2
sudo apt install python3-rpi.gpio
sudo apt install python3-serial
sudo apt install python3-opencv
```
### 2.3. Create a ROS2 workspace:
```bash
mkdir -p ~/ros2_hex_tutorial_ws/src
cd ~/ros2_ws/src
```
### 2.4. Clone the repository into src directory:
```bash
git clone https://github.com/carlobeni/hexapod_pkg.git
```
### 2.5. Build the workspace:
```bash
cd ~/ros2_hex_tutorial_ws
colcon build --symlink-install
```
### 2.6. Source the workspace permanently:
```bash
echo "source ~/ros2_hex_tutorial_ws/install/setup.bash" >> ~/.bashrc
```

## 3. Arduino Mega installation
Load by Arduino IDE the sketch `hexapod_pkg/arduino/Hexapod2.ino`





