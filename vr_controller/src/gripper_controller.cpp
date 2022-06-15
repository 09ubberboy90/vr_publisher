//BSD 3-Clause License
//
//Copyright (c) 2021, Florent Audonnet
//All rights reserved.
//
//Redistribution and use in source and binary forms, with or without
//modification, are permitted provided that the following conditions are met:
//
//1. Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
//2. Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
//3. Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
//THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
//IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
//DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
//FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
//DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
//SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
//OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/bool.hpp>

using std::placeholders::_1;

enum gripper_state
{
    opened = 35,
    closed = 0
};

bool wait_for_exec(moveit::planning_interface::MoveGroupInterface *move_group)
{
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    for (int i = 0; i < 10; i++)
    {
        // 10 tries to plan otherwise give up
        bool success = (move_group->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Plan %d %s", i, success ? "SUCCEEDED" : "FAILED");

        if (success)
        {
            move_group->execute(plan);
            return true;
        }
    }
    auto pose = move_group->getPoseTarget().pose.position;
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to find a valid path to %f, %f, %f", pose.x, pose.y, pose.z);
    return false;
}

bool change_gripper(moveit::planning_interface::MoveGroupInterface *hand_move_group, gripper_state state)
{
    auto current_state = hand_move_group->getCurrentState();
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(current_state->getJointModelGroup("t42_gripper"), joint_group_positions);
    float pose = (float)state / 1000;
    joint_group_positions[0] = pose;
    joint_group_positions[1] = pose;
    hand_move_group->setJointValueTarget(joint_group_positions);
    return wait_for_exec(hand_move_group);
}

class VrTriggerSubscriber : public rclcpp::Node
{
public:
    VrTriggerSubscriber(): Node("vr_trigger_sub")
    {
        current_state = gripper_state::opened;
        subscription_ = this->create_subscription<std_msgs::msg::Bool>(
        "RightHand/trigger", 10, std::bind(&VrTriggerSubscriber::topic_callback, this, _1));

    }
    gripper_state current_state;

private:
    void topic_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data && current_state !=gripper_state::opened)
        {
            RCLCPP_INFO(rclcpp::get_logger("vr_trigger_sub"), "Changing gripper state");
            current_state = gripper_state::opened;
        }
        else if (current_state !=gripper_state::closed)
        {
            RCLCPP_INFO(rclcpp::get_logger("vr_trigger_sub"), "Changing gripper state");
            //change_gripper(&hand_move_group, gripper_state::closed);
            current_state = gripper_state::closed;
        }
        
    }
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{

    rclcpp::init(argc, argv);
    // For current state monitor
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_node = rclcpp::Node::make_shared("move_group_interface", node_options);
    auto vr_trigger = std::make_shared<VrTriggerSubscriber>();
    // For current state monitor
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(move_group_node);
    executor.add_node(vr_trigger);
    std::thread([&executor]() { executor.spin(); }).detach();
    moveit::planning_interface::MoveGroupInterface hand_move_group(std::shared_ptr<rclcpp::Node>(move_group_node), "t42_gripper");
    gripper_state current_state = opened;

    hand_move_group.setMaxVelocityScalingFactor(1.0);
    hand_move_group.setMaxAccelerationScalingFactor(1.0);
    while (true)
    {
        if (vr_trigger->current_state == opened && current_state == closed)
        {
            change_gripper(&hand_move_group, gripper_state::opened);
            current_state = opened;
        }
        else if (vr_trigger->current_state == closed && current_state == opened)
        {
            change_gripper(&hand_move_group, gripper_state::closed);
            current_state = closed;
        }
        continue;
    }
    
    rclcpp::shutdown();
    return 0;
}