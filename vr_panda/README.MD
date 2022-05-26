# Commands for velocity
ros launch ur_bringup ur_control.launch.py ur_type:=ur10 robot_ip:=yyy.yyy.yyy.yyy use_fake_hardware:=true launch_rviz:=true
ros run vr_publisher vr_publish
ros run vr_controller rviz_vr_visual (optional)
ros launch vr_panda vr_controller.launch.py