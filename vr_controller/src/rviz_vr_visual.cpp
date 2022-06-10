// BSD 3-Clause License
//
// Copyright (c) 2021, Florent Audonnet
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "geometry_msgs/msg/pose.hpp"

#include <rviz_visual_tools/rviz_visual_tools.hpp>

#include <chrono>
#include <map>
#include <string>
#include <tf2_eigen/tf2_eigen.h>
#include <vector>

class MinimalSubscriber : public rclcpp::Node
{
public:
    MinimalSubscriber()
        : Node("minimal_subscriber")
    {
        visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("world", "/rviz_visual_markers", this));
        visual_tools_->deleteAllMarkers();

        std::function<void(const std::shared_ptr<geometry_msgs::msg::Pose>)>
            fcn1 = std::bind(&MinimalSubscriber::topic_callback, this, std::placeholders::_1, "hmd");
        subscription_hmd = this->create_subscription<geometry_msgs::msg::Pose>(
            "/hmd/pos", 10, fcn1);

        std::function<void(const std::shared_ptr<geometry_msgs::msg::Pose>)>
            fcn2 = std::bind(&MinimalSubscriber::topic_callback, this, std::placeholders::_1, "left");
        subscription_left = this->create_subscription<geometry_msgs::msg::Pose>(
            "/LeftHand/pos", 10, fcn2);

        std::function<void(const std::shared_ptr<geometry_msgs::msg::Pose>)>
            fcn3 = std::bind(&MinimalSubscriber::topic_callback, this, std::placeholders::_1, "right");
        subscription_right = this->create_subscription<geometry_msgs::msg::Pose>(
            "/RightHand/pos", 10, fcn3);

        std::function<void(const std::shared_ptr<geometry_msgs::msg::Pose>)>
            fcn4 = std::bind(&MinimalSubscriber::topic_callback, this, std::placeholders::_1, "RTracker");
        subscription_Tright = this->create_subscription<geometry_msgs::msg::Pose>(
            "/Tracker1/pos", 10, fcn4);

        std::function<void(const std::shared_ptr<geometry_msgs::msg::Pose>)>
            fcn5 = std::bind(&MinimalSubscriber::topic_callback, this, std::placeholders::_1, "LTracker");
        subscription_Tleft = this->create_subscription<geometry_msgs::msg::Pose>(
            "/Tracker2/pos", 10, fcn5);
    }

private:
    void topic_callback(const geometry_msgs::msg::Pose::SharedPtr msg, std::string name)
    {
        auto pose = Eigen::Isometry3d(
            Eigen::Translation3d(msg->position.z, msg->position.x, msg->position.y) *
            Eigen::Quaterniond(msg->orientation.w,
                               msg->orientation.z,
                               msg->orientation.x,
                               msg->orientation.y));
        
        // Publish arrow vector of pose
        visual_tools_->resetMarkerCounts();
        // visual_tools_->deleteMarker(name);
        publishLabelHelper(pose, name);
        visual_tools_->publishAxis(pose,rviz_visual_tools::MEDIUM,name);

        // Don't forget to trigger the publisher!
        visual_tools_->trigger();
    }

    void publishLabelHelper(const Eigen::Isometry3d &pose, const std::string &label) const
    {
        Eigen::Isometry3d pose_copy = pose;
        pose_copy.translation().x() -= 0.2;
        visual_tools_->publishText(pose_copy, label, rviz_visual_tools::WHITE, rviz_visual_tools::XXLARGE, false);
    }

    rviz_visual_tools::RvizVisualToolsPtr visual_tools_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscription_hmd;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscription_left;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscription_right;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscription_Tleft;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscription_Tright;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalSubscriber>());
    rclcpp::shutdown();
    return 0;
}