nav2 custom config files (DEPRICATED)
=======================
This part of the code, meaning setting up a custom nav2 stack and the 
associated config files, is depricated (and does not work). DO NOT USE IT, 
its only purpose is for future development.

The current issue is described in [this Robotics Stack Exchange Question](https://robotics.stackexchange.com/questions/113468/no-map-received-timed-out-waiting-for-transform-from-base-footprint-to-map).

Alternative
=======================
The temporary alternative is to use the default nav2 stack together with SLAM 
Toolbox. To start everything which is necessary to run the individual robot 
behaviour (path planning):
Start SLAM Toolbox (this requires publishing LIDAR values on the /scan topic):
```
ros2 launch slam_toolbox online_async_launch.py
```
Start the nav2 stack (This requires SLAM Toolbox and odometry being published 
on /odom topic):
```
ros2 launch nav2_bringup navigation_launch.py
```
