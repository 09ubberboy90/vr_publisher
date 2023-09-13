# vr_publisher

Getting the 3D position and rotation of any connected controller to SteamVR and publishes to ROS2.

Part of the telesim_pnp packages suite available here for the master or here for just the UR3/T42 part.

## How to run

Run SteamVR and make sure to reset the origin of the headset.

Build and source the workspace using `colcon build`

Run this package by using `ros2 run vr_publish vr_publish`

