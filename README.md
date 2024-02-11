# Visual Servoing with BlueROV

## Overview
This project, guided by Prof. C. Dune and Prof. V. Hugle at the University of Toulon, focuses on implementing visual servoing techniques with the BlueROV underwater vehicle. The core of this project lies in blob detection for navigation and interaction within underwater environments, utilizing ROS (Robot Operating System) for communication and control. Key components include script development for camera parameter calibration, image processing, and ROS message broadcasting.

## Key Features
- **Camera Parameters**: Script for converting camera measurements from pixels to meters, enabling accurate real-world interpretations of visual data.
- **Image Processing**: A template Python script (`image_processing_mir.py`) for image display, blob detection, and ROS message broadcasting for tracked and desired points.
- **Blob Detection**: Implementation of blob detection algorithms focused on identifying specific features in the underwater environment, using the BlueROV as the testing platform.

## System Requirements
- Compatible with Linux Ubuntu 18.04 LTS or 20.04 LTS
- ROS Melodic (for Ubuntu 18.04) or Neotic (for Ubuntu 20.04)

## Setup Instructions

### Initial Setup
1. Install Ubuntu on a separate partition if necessary.
2. Follow the ROS installation guide specific to your Ubuntu version at [ROS Installation](http://wiki.ros.org/Installation/Ubuntu).

### ROS and Dependencies Installation
For Ubuntu 18.04 with ROS Melodic (replace `melodic` with `noetic` for Ubuntu 20.04 and `python` with `python3`):
```bash
sudo apt-get install -y cmake python-catkin-pkg python-empy python-nose python-setuptools libgtest-dev build-essential openssh-server
sudo apt-get install python-wstool python-rosinstall-generator python-catkin-tools
sudo apt-get install ros-melodic-joy
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
```

### MAVROS Installation
- **Local Installation** (recommended): Copy the `autonomous_rov`, `mavros`, and `mavlink` packages provided in the TGZ files into `~/catkin_ws/src`.

  Then build the packages:
  ```bash
  catkin build
  # For verbose output or if encountering issues, use:
  catkin build -j1 -v
  ```

- **Network Installation**: Alternatively, you can install MAVROS from the network, but ensure compatibility with the embedded version on BlueROV.
  ```bash
  cd ~/catkin_ws
  catkin init
  wstool init ~/catkin_ws/src
  rosinstall_generator --upstream mavros | tee /tmp/mavros.rosinstall
  rosinstall_generator mavlink | tee -a /tmp/mavros.rosinstall
  wstool merge -t src /tmp/mavros.rosinstall
  wstool update -t src
  rosdep install --from-paths src --ignore-src --rosdistro melodic -y # Replace 'melodic' with 'noetic' for Ubuntu 20.04
  ```

### Modifying `OverrideRCIn.msg`
For compatibility, modify the `OverrideRCIn.msg` in `~/catkin_ws/src/mavros/mavros_msgs` as follows:
- Change `uint16[18] channels` to `uint16[8] channels`.

## Usage
After setting up your environment and building the necessary packages, you can run the `image_processing_mir.py` script to start blob detection and point tracking. Ensure that the BlueROV is connected and properly configured with ROS to receive commands and broadcast messages.

## Contributions and Acknowledgments
This project was developed under the supervision of Prof. C. Dune, Prof. V. Hugle and with contributions from the robotics community at the University of Toulon. Special thanks to the ROS and MAVROS developers for their invaluable tools and documentation.
