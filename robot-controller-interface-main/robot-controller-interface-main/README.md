Robot Controller Interface
=======================

About
-----------------------
This repository contains the executable code intended for running on the 
individual robots (on the raspberry pi), termed Individual Robot Behaviour, 
and the accompanying Raspberry Pi part of the Hardware interface API.

### Built with
The Robot Controller Interface is built with the following:

- [ROS2 Humble](https://docs.ros.org/en/humble/index.html)
- [nav2](https://docs.nav2.org/)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)

Getting started
-----------------------

### Prerequisites
On Ubuntu and other Debian derivatives, CMake can be installed with:
```
sudo apt install build-essential cmake
```

For ros2 humble see: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

For nav2 see: https://docs.nav2.org/getting_started/index.html#installation

For SLAM Toolbox see: https://docs.nav2.org/tutorials/docs/navigation2_with_slam.html

### Installation
1. Clone the repository:
```
git clone https://github.com/DVA490-474-Project-Course/robot-controller-interface.git
```
2. Navigate to the project directory:
```
cd robot-controller-interface
```
3. Create a build directory and navigate to it:
```
mkdir build & cd build
```
4. Build the source code:
```
cmake ..
make
```
5. Locate the binaries which should be stored in bin:
```
cd ../bin
```
6. Execute the desired binaries.

Usage
-----------------------

To use individual robot behaviour (path planning etc), then the following 
preconditions must be started:
Start SLAM Toolbox (this requires publishing LIDAR values on the /scan topic):
```
ros2 launch slam_toolbox online_async_launch.py
```
Start the nav2 stack (This requires SLAM Toolbox and odometry being published 
on /odom topic):
```
ros2 launch nav2_bringup navigation_launch.py
```

Examples
-----------------------

### Path planning
An example showing the functionality of the path planning can be seen by doing 
the following (this also requires the Gazebo simulation environment, see 
https://docs.nav2.org/getting_started/index.html):
Start Gazebo:
```
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```
Start SLAM Toolbox:
```
ros2 launch slam_toolbox online_async_launch.py
```
Start the nav2 stack:
```
ros2 launch nav2_bringup navigation_launch.py
```
Run the following binary available in the robot-controller-interface/bin/ 
(bin in this projects root):
```
./main_test_exe --gtest_also_run_disabled_tests --gtest_filter=DwbControllerTest.DISABLED_SendTargetPoseTest
```

### Angle towards target and shoot (ShootSetup)
No examples available.

Roadmap
-----------------------
API:
- [ ] Develop Hardware Interface

Individual Robot Behaviour (executable):
- [x] Develop Path planning 
- [x] Develop robot ability to shoot
- [ ] nav2 stack configuration
- [ ] Integrate Hardware Interface
- [ ] Ability to send data
- [ ] Ability to receive data
- [ ] Initilization of robot

Design diagrams
-----------------------
Design diagrams/files can be found in the [docs](/docs) directory. Additionally 
they are available on:
- [Hardware Interface](https://www.mermaidchart.com/raw/11c442f5-192c-4ac3-b61e-867a3e2ca6ea?theme=dark&version=v0.1&format=svg)
- [Individual Robot Behaviour](https://www.mermaidchart.com/raw/dc459e07-4c98-46b8-8ac0-41c56aa6950f?theme=dark&version=v0.1&format=svg)

License
-----------------------
Distributed under the MIT License. See [License](/LICENSE) for more information.

Contributors and contact
-----------------------
- Aaiza Aziz Khan: akn23018@student.mdu.se
- Carl Larsson: cln20001@student.mdu.se
- Shruthi Puthiya Kunnon: spn23001@student.mdu.se
- Emil Åberg: eag24002@student.mdu.se
