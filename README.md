# vr_publisher

Getting the 3D position and rotation of any connected controller to SteamVR and publishes to ROS2.

Part of the telesim_pnp packages suite available [here](https://github.com/09ubberboy90/telesim_pnp.git).

## How to run

Run SteamVR and make sure to reset the origin of the headset.

Build and source the workspace using `colcon build`

Run this package by using `ros2 run vr_publish vr_publish`

