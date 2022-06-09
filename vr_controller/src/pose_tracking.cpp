/*******************************************************************************
 *      Title     : pose_tracking_example.cpp
 *      Project   : moveit_servo
 *      Created   : 09/04/2020
 *      Author    : Adam Pettinger
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

#include <moveit_servo/make_shared_from_pool.h>
#include <moveit_servo/pose_tracking.h>
#include <moveit_servo/servo.h>
#include <moveit_servo/servo_parameters.h>
#include <moveit_servo/status_codes.h>
#include <thread>
using namespace std::chrono_literals;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_servo.pose_tracking_demo");

rclcpp::Node::SharedPtr node_;
rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_cmd_pub_;
geometry_msgs::msg::PoseStamped target_pose;

class VrSubscriber : public rclcpp::Node
{
public:
    VrSubscriber()
        : Node("vr_subscriber")
    {
        subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
            // "/tracker/Tracker1", 10, std::bind(&VrSubscriber::execute_goal, this, std::placeholders::_1));
            "/controller/RightHand", 10, std::bind(&VrSubscriber::execute_goal, this, std::placeholders::_1));
    }
    geometry_msgs::msg::Pose::SharedPtr pose;

private:
    void execute_goal(geometry_msgs::msg::Pose::SharedPtr msg)
    {
        pose = msg;
    }
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscription_;
};

// Class for monitoring status of moveit_servo
class StatusMonitor
{
public:
    StatusMonitor(const rclcpp::Node::SharedPtr &node, const std::string &topic)
    {
        sub_ = node->create_subscription<std_msgs::msg::Int8>(
            topic, rclcpp::SystemDefaultsQoS(), std::bind(&StatusMonitor::statusCB, this, std::placeholders::_1));
    }

private:
    void statusCB(const std_msgs::msg::Int8::ConstSharedPtr msg)
    {
        moveit_servo::StatusCode latest_status = static_cast<moveit_servo::StatusCode>(msg->data);
        if (latest_status != status_)
        {
            status_ = latest_status;
            const auto &status_str = moveit_servo::SERVO_STATUS_CODE_MAP.at(status_);
            RCLCPP_INFO_STREAM(LOGGER, "Servo status: " << status_str);
        }
    }

    moveit_servo::StatusCode status_ = moveit_servo::StatusCode::INVALID;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr sub_;
};

/**
 * Instantiate the pose tracking interface.
 * Send a pose slightly different from the starting pose
 * Then keep updating the target pose a little bit
 */

void publishCommands(std::shared_ptr<VrSubscriber> vr_subscriber)
{
    // msg->header.frame_id = "base_link";
    if (vr_subscriber->pose)
    {
        target_pose.header.stamp = node_->now();
        target_pose.pose.position.x = vr_subscriber->pose->position.z;
        target_pose.pose.position.y = vr_subscriber->pose->position.x;
        target_pose.pose.position.z = vr_subscriber->pose->position.y;

        target_pose.pose.orientation.w = vr_subscriber->pose->orientation.w;
        target_pose.pose.orientation.x = -vr_subscriber->pose->orientation.z;
        target_pose.pose.orientation.y = -vr_subscriber->pose->orientation.x;
        target_pose.pose.orientation.z = -vr_subscriber->pose->orientation.y;
        pose_cmd_pub_->publish(target_pose);
    }
    else
    {
        RCLCPP_INFO(rclcpp::get_logger("moveit_servo.pose_tracking_demo"), "Pose is not defined");
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    node_ = rclcpp::Node::make_shared("pose_tracking_demo");

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node_);
    std::thread executor_thread([&executor]()
                                { executor.spin(); });

    auto servo_parameters = moveit_servo::ServoParameters::makeServoParameters(node_);

    if (servo_parameters == nullptr)
    {
        RCLCPP_FATAL(LOGGER, "Could not get servo parameters!");
        exit(EXIT_FAILURE);
    }

    // Load the planning scene monitor
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor;
    planning_scene_monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node_, "robot_description");
    if (!planning_scene_monitor->getPlanningScene())
    {
        RCLCPP_ERROR_STREAM(LOGGER, "Error in setting up the PlanningSceneMonitor.");
        exit(EXIT_FAILURE);
    }

    planning_scene_monitor->providePlanningSceneService();
    planning_scene_monitor->startSceneMonitor();
    planning_scene_monitor->startWorldGeometryMonitor(
        planning_scene_monitor::PlanningSceneMonitor::DEFAULT_COLLISION_OBJECT_TOPIC,
        planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_WORLD_TOPIC,
        false /* skip octomap monitor */);
    planning_scene_monitor->startStateMonitor(servo_parameters->joint_topic);
    planning_scene_monitor->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);

    // Wait for Planning Scene Monitor to setup
    if (!planning_scene_monitor->waitForCurrentRobotState(node_->now(), 5.0 /* seconds */))
    {
        RCLCPP_ERROR_STREAM(LOGGER, "Error waiting for current robot state in PlanningSceneMonitor.");
        exit(EXIT_FAILURE);
    }

    // Create the pose tracker
    moveit_servo::PoseTracking tracker(node_, servo_parameters, planning_scene_monitor);

    // Make a publisher for sending pose commands
    pose_cmd_pub_ =
        node_->create_publisher<geometry_msgs::msg::PoseStamped>("target_pose", rclcpp::SystemDefaultsQoS());

    // Subscribe to servo status (and log it when it changes)
    StatusMonitor status_monitor(node_, servo_parameters->status_topic);

    Eigen::Vector3d lin_tol{0.001, 0.001, 0.001};
    double rot_tol = 0.01;

    // Get the current EE transform
    geometry_msgs::msg::TransformStamped current_ee_tf;
    tracker.getCommandFrameTransform(current_ee_tf);

    // Convert it to a Pose
    target_pose.header.frame_id = current_ee_tf.header.frame_id;
    target_pose.pose.position.x = current_ee_tf.transform.translation.x;
    target_pose.pose.position.y = current_ee_tf.transform.translation.y;
    target_pose.pose.position.z = current_ee_tf.transform.translation.z;
    target_pose.pose.orientation = current_ee_tf.transform.rotation;

    // Modify it a little bit
    target_pose.pose.position.x += 0.1;

    // resetTargetPose() can be used to clear the target pose and wait for a new one, e.g. when moving between multiple
    // waypoints
    tracker.resetTargetPose();
    auto vr_subscriber = std::make_shared<VrSubscriber>();
    executor.add_node(vr_subscriber);

    rclcpp::TimerBase::SharedPtr timer = node_->create_wall_timer(50ms,  [vr_subscriber]() -> void { publishCommands(vr_subscriber); });

    // Publish target pose
    target_pose.header.stamp = node_->now();
    pose_cmd_pub_->publish(target_pose);

    // Run the pose tracking in a new thread

    moveit_servo::PoseTrackingStatusCode tracking_status =
        tracker.moveToPose(lin_tol, rot_tol, 0.1 /* target pose timeout */);
    RCLCPP_INFO_STREAM(LOGGER, "Pose tracker exited with status: "
                                   << moveit_servo::POSE_TRACKING_STATUS_CODE_MAP.at(tracking_status)); 

    // Kill executor thread before shutdown
    executor.cancel();
    executor_thread.join();

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}