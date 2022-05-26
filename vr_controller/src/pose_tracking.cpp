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

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_servo.pose_tracking_demo");

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

class VrSubscriber : public rclcpp::Node
{
public:
    VrSubscriber()
        : Node("vr_subscriber")
    {
        subscription_pose = this->create_subscription<geometry_msgs::msg::Pose>(
            "/controller/RightHand", 10, std::bind(&VrSubscriber::receive_pose, this, std::placeholders::_1));
        // subscription_twist = this->create_subscription<geometry_msgs::msg::Twist>(
        //     "/controller/RightHand/velc", 10, std::bind(&VrSubscriber::receive_twist, this, std::placeholders::_1));
        target_pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("target_pose", rclcpp::SystemDefaultsQoS());
        auto node = std::shared_ptr<rclcpp::Node>(std::move(this));
        planning_scene_monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node, "robot_description");
        if (!planning_scene_monitor->getPlanningScene())
        {
            RCLCPP_ERROR_STREAM(LOGGER, "Error in setting up the PlanningSceneMonitor.");
            exit(EXIT_FAILURE);
        }
        RCLCPP_INFO_STREAM(LOGGER, "Pose tracker exited with status: ");


        auto servo_parameters = moveit_servo::ServoParameters::makeServoParameters(node);

        if (servo_parameters == nullptr)
        {
            RCLCPP_FATAL(LOGGER, "Could not get servo parameters!");
            exit(EXIT_FAILURE);
        }
        RCLCPP_INFO_STREAM(LOGGER, "Pose tracker exited with status: ");

        // Load the planning scene monitor

        planning_scene_monitor->providePlanningSceneService();
        planning_scene_monitor->startSceneMonitor();
        planning_scene_monitor->startWorldGeometryMonitor(
            planning_scene_monitor::PlanningSceneMonitor::DEFAULT_COLLISION_OBJECT_TOPIC,
            planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_WORLD_TOPIC,
            false /* skip octomap monitor */);
        planning_scene_monitor->startStateMonitor(servo_parameters->joint_topic);
        planning_scene_monitor->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);
        RCLCPP_INFO_STREAM(LOGGER, "Pose tracker exited with status: ");

        // Wait for Planning Scene Monitor to setup
        if (!planning_scene_monitor->waitForCurrentRobotState(this->now(), 5.0 /* seconds */))
        {
            RCLCPP_ERROR_STREAM(LOGGER, "Error waiting for current robot state in PlanningSceneMonitor.");
            exit(EXIT_FAILURE);
        }

        // Create the pose tracker
        moveit_servo::PoseTracking tracker(node, servo_parameters, planning_scene_monitor);
            
        RCLCPP_INFO_STREAM(LOGGER, "Pose tracker exited with status: ");

        // Subscribe to servo status (and log it when it changes)
        StatusMonitor status_monitor(node, servo_parameters->status_topic);

        Eigen::Vector3d lin_tol{0.001, 0.001, 0.001};
        double rot_tol = 0.01;
        RCLCPP_INFO_STREAM(LOGGER, "Pose tracker exited with status: ");

        // Get the current EE transform
        geometry_msgs::msg::TransformStamped current_ee_tf;
        tracker.getCommandFrameTransform(current_ee_tf);
        header = current_ee_tf.header.frame_id;
        RCLCPP_INFO_STREAM(LOGGER, "Pose tracker exited with status: ");

        std::thread move_to_pose_thread([&tracker, &lin_tol, &rot_tol] {
            moveit_servo::PoseTrackingStatusCode tracking_status =
                tracker.moveToPose(lin_tol, rot_tol, 0.1 /* target pose timeout */);
            RCLCPP_INFO_STREAM(LOGGER, "Pose tracker exited with status: "
                                        << moveit_servo::POSE_TRACKING_STATUS_CODE_MAP.at(tracking_status));
        });
        RCLCPP_INFO_STREAM(LOGGER, "Pose tracker exited with status: ");

    }
    // geometry_msgs::msg::Twist::SharedPtr twist;
    geometry_msgs::msg::Pose::SharedPtr pose;

private:
    std::string header;
    std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>> target_pose_pub;
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor;

    // void receive_twist(geometry_msgs::msg::Twist::SharedPtr msg)
    // {
    //     twist = msg;
    // }
    // rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_twist;
    void receive_pose(geometry_msgs::msg::Pose::SharedPtr msg)
    {
        try
        {
            pose = msg;
            geometry_msgs::msg::PoseStamped target_pose;

            target_pose.header.frame_id = header;
            Eigen::Quaternionf q = Eigen::AngleAxisf(3.14, Eigen::Vector3f::UnitX()) * Eigen::AngleAxisf(0, Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(0.785, Eigen::Vector3f::UnitZ());
            pose->orientation.w = q.w();
            pose->orientation.x = q.x();
            pose->orientation.y = q.y();
            pose->orientation.z = q.z();
            target_pose.pose = *pose;
            if (target_pose_pub)
            {
                target_pose_pub->publish(target_pose);
            }
        }
        catch (...)
        {
            return;
        }
    }
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscription_pose;
};
/**
 * Instantiate the pose tracking interface.
 * Send a pose slightly different from the starting pose
 * Then keep updating the target pose a little bit
 */
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor executor;
    auto subscriber = std::make_shared<VrSubscriber>();
    executor.add_node(subscriber);

    executor.spin();
    
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}