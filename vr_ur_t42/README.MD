# Commands for velocity
ros2 launch ur_bringup ur_control.launch.py ur_type:=ur5e robot_ip:=yyy.yyy.yyy.yyy use_fake_hardware:=true launch_rviz:=false initial_joint_controller:=joint_trajectory_controller
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e launch_rviz:=true launch_servo:=false
ros run vr_publisher vr_publish
ros run vr_controller rviz_vr_visual (optional)
ros launch vr_panda vr_controller.launch.py