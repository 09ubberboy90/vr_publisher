/*******************************************************************************
 *      Title     : pose_tracking_example.cpp -> vr_controller
 *      Project   : moveit_servo -> simple_arm_controller
 *      Created   : 09/04/2020
 *      Author    : Adam Pettinger, Florent Audonnet
 *
 * BSD 3-Clause License
 *
 * Copyright (c) 2019, Los Alamos National Security, LLC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/int8.hpp>
#include <rclcpp/rclcpp.hpp>

#include <moveit_servo/servo_parameters.h>
#include <moveit_servo/servo.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <thread>

#include <memory>
#include <string>
#include <vector>

using namespace std::chrono_literals;

rclcpp::Node::SharedPtr node_;
rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_cmd_pub_;
size_t count_ = 0;


class VrSubscriber : public rclcpp::Node
{
public:
    VrSubscriber()
        : Node("vr_subscriber")
    {
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            // "/tracker/Tracker3/vel", 10, std::bind(&VrSubscriber::execute_goal, this, std::placeholders::_1));
            "/RightHand/vel", 10, std::bind(&VrSubscriber::execute_goal, this, std::placeholders::_1));
    }
    geometry_msgs::msg::Twist::SharedPtr twist;
private:
    void execute_goal(geometry_msgs::msg::Twist::SharedPtr msg)
    {
        twist = msg;
    }
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
};

void publishCommands(std::shared_ptr<VrSubscriber> vr_subscriber)
{
    auto msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
    msg->header.stamp = node_->now();
    // msg->header.frame_id = "base_link";
    if (vr_subscriber->twist)
    {
        msg->twist = *vr_subscriber->twist;
        twist_cmd_pub_->publish(std::move(msg));
    }
    else{
        RCLCPP_INFO(rclcpp::get_logger("moveit_servo.pose_tracking_demo"), "Twist is not defined");
    }
    
}

int main(int argc, char **argv)
{

    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;

    // This is false for now until we fix the QoS settings in moveit to enable intra process comms
    node_options.use_intra_process_comms(false);
    node_ = std::make_shared<rclcpp::Node>("servo_demo_node", node_options);
    // Pause for RViz to come up. This is necessary in an integrated demo with a single launch file
    // rclcpp::sleep_for(std::chrono::seconds(4));

    // // Create the planning_scene_monitor. We need to pass this to Servo's constructor, and we should set it up first
    // // before initializing any collision objects
    auto planning_scene_monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
        node_, "robot_description", "planning_scene_monitor");

    // Here we make sure the planning_scene_monitor is updating in real time from the joint states topic
    if (planning_scene_monitor->getPlanningScene())
    {
        planning_scene_monitor->startStateMonitor("/joint_states");
        planning_scene_monitor->setPlanningScenePublishingFrequency(25);
        planning_scene_monitor->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE,
                                                                "/moveit_servo/publish_planning_scene");
        planning_scene_monitor->startSceneMonitor();
        planning_scene_monitor->providePlanningSceneService();
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("moveit_servo.pose_tracking_demo"), "Planning scene not configured");
        return EXIT_FAILURE;
    }
      // These are the publishers that will send commands to MoveIt Servo. Two command types are supported: JointJog
    // messages which will directly jog the robot in the joint space, and TwistStamped messages which will move the
    // specified link with the commanded Cartesian velocity. In this demo, we jog the end effector link.
    // twist_cmd_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>("servo_demo_node/delta_twist_cmds", 10);
    twist_cmd_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>("/servo_node/delta_twist_cmds", 10);


    // Initializing Servo
    // ^^^^^^^^^^^^^^^^^^
    // Servo requires a number of parameters to dictate its behavior. These can be read automatically by using the
    // :code:`makeServoParameters` helper function
    auto servo_parameters = moveit_servo::ServoParameters::makeServoParameters(node_);
    if (!servo_parameters)
    {
        RCLCPP_FATAL(rclcpp::get_logger("moveit_servo.pose_tracking_demo"), "Failed to load the servo parameters");
        return EXIT_FAILURE;
    }

    // Initialize the Servo C++ interface by passing a pointer to the node, the parameters, and the PSM
    auto servo = std::make_unique<moveit_servo::Servo>(node_, servo_parameters, planning_scene_monitor);

    // You can start Servo directly using the C++ interface. If launched using the alternative ServoNode, a ROS
    // service is used to start Servo. Before it is started, MoveIt Servo will not accept any commands or move the robot
    servo->start();

    auto vr_subscriber = std::make_shared<VrSubscriber>();
    // Sending Commands
    // ^^^^^^^^^^^^^^^^
    // For this demo, we will use a simple ROS timer to send joint and twist commands to the robot
    
    rclcpp::TimerBase::SharedPtr timer = node_->create_wall_timer(50ms,  [vr_subscriber]() -> void { publishCommands(vr_subscriber); });

    // CALL_SUB_TUTORIAL publishCommands

    // We use a multithreaded executor here because Servo has concurrent processes for moving the robot and avoiding collisions
    auto executor = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();
    executor->add_node(node_);
    executor->add_node(vr_subscriber);
    executor->spin();
    rclcpp::shutdown();
    return 0;
}
