cmake_minimum_required(VERSION 3.5)
project(vr_controller)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_BUILD_TYPE Debug)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(rviz_visual_tools REQUIRED)
find_package(moveit_servo REQUIRED)
find_package(moveit REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(backward_ros REQUIRED)
find_package(std_msgs REQUIRED)
  
install(DIRECTORY
  DESTINATION share/${PROJECT_NAME}/
)
include_directories(include)
add_compile_options(-g)

add_executable(rviz_vr_visual src/rviz_vr_visual.cpp)
ament_target_dependencies(rviz_vr_visual
  rclcpp
  tf2_eigen
  rviz_visual_tools
  geometry_msgs
)

add_executable(gripper_controller src/gripper_controller.cpp)
ament_target_dependencies(gripper_controller
  rclcpp
  moveit_ros_planning_interface
  moveit
  std_msgs
)

add_executable(pose_tracking src/pose_tracking.cpp)
ament_target_dependencies(pose_tracking
  rclcpp
  moveit_servo
  geometry_msgs
)

add_executable(vr_controller src/vr_controller.cpp)
ament_target_dependencies(vr_controller
  rclcpp
  moveit_servo
  geometry_msgs
)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS rviz_vr_visual vr_controller pose_tracking gripper_controller
  DESTINATION lib/${PROJECT_NAME}
)
    
ament_package()
