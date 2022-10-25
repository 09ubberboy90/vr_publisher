#BSD 3-Clause License
#
#Copyright (c) 2021, Florent Audonnet
#All rights reserved.
#
#Redistribution and use in source and binary forms, with or without
#modification, are permitted provided that the following conditions are met:
#
#1. Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
#2. Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
#3. Neither the name of the copyright holder nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.
#
#THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
#DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
#FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
#DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
#SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
#OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
from typing import Tuple

import sys
from datetime import datetime
import openvr
import pyquaternion as pyq
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, TwistStamped, Vector3
from std_msgs.msg import Bool
from collections import defaultdict
import numpy as np

# https: // gist.github.com/awesomebytes/75daab3adb62b331f21ecf3a03b3ab46
import math

def convert_to_euler(pose_mat):
    yaw = 180 / math.pi * math.atan2(pose_mat[1][0], pose_mat[0][0])
    pitch = 180 / math.pi * math.atan2(pose_mat[2][0], pose_mat[0][0])
    roll = 180 / math.pi * math.atan2(pose_mat[2][1], pose_mat[2][2])
    x = pose_mat[0][3]
    y = pose_mat[1][3]
    z = pose_mat[2][3]
    return [[x,y,z],[yaw,pitch,roll]]

def convert_to_quaternion(pose_mat):
    # Per issue #2, adding a abs() so that sqrt only results in real numbers
    # r_w = math.sqrt(abs(1+pose_mat[0][0]+pose_mat[1][1]+pose_mat[2][2]))/2
    # r_x = (pose_mat[2][1]-pose_mat[1][2])/(4*r_w)
    # r_y = (pose_mat[0][2]-pose_mat[2][0])/(4*r_w)
    # r_z = (pose_mat[1][0]-pose_mat[0][1])/(4*r_w)
    
    q_w = math.sqrt(max(0, 1 + pose_mat[0][0] + pose_mat[1][1] + pose_mat[2][2])) / 2;
    q_x = math.sqrt(max(0, 1 + pose_mat[0][0] - pose_mat[1][1] - pose_mat[2][2])) / 2;
    q_y = math.sqrt(max(0, 1 - pose_mat[0][0] + pose_mat[1][1] - pose_mat[2][2])) / 2;
    q_z = math.sqrt(max(0, 1 - pose_mat[0][0] - pose_mat[1][1] + pose_mat[2][2])) / 2;
    q_x = math.copysign(q_x, pose_mat[2][1] - pose_mat[1][2]);
    q_y = math.copysign(q_y, pose_mat[0][2] - pose_mat[2][0]);
    q_z = math.copysign(q_z, pose_mat[1][0] - pose_mat[0][1]);

    x = pose_mat[0][3]
    y = pose_mat[1][3]
    z = pose_mat[2][3]
    return [[x,y,z],[q_w,q_x,q_y,q_z]]

def from_controller_state_to_dict(pControllerState):
    # docs: https://github.com/ValveSoftware/openvr/wiki/IVRSystem::GetControllerState
    controller_state = {}
    controller_state['unPacketNum'] = pControllerState.unPacketNum
    # on trigger .y is always 0.0 says the docs
    controller_state['trigger'] = pControllerState.rAxis[3].x
    try:    
        print(f"1 : {pControllerState.rAxis0} {pControllerState.rAxis0}")
    except:
        pass
    try:    
        print(f"2 : {pControllerState.rAxis1.x} {pControllerState.rAxis1.y}")
    except:
        pass
    try:    
        print(f"3 : {pControllerState.rAxis2.x} {pControllerState.rAxis2.y}")
    except:
        pass
    try:    
        print(f"4 : {pControllerState.rAxis3.x} {pControllerState.rAxis3.y}")
    except:
        pass
    # 0.0 on trigger is fully released
    # -1.0 to 1.0 on joystick and trackpads
    controller_state['trackpad_x'] = pControllerState.rAxis[0].x
    controller_state['trackpad_y'] = pControllerState.rAxis[0].y
    # These are published and always 0.0
    # for i in range(2, 5):
    #     controller_state['unknowns_' + str(i) + '_x'] = pControllerState.rAxis[i].x
    #     controller_state['unknowns_' + str(i) + '_y'] = pControllerState.rAxis[i].y
    controller_state['ulButtonPressed'] = pControllerState.ulButtonPressed
    controller_state['ulButtonTouched'] = pControllerState.ulButtonTouched
    # To make easier to understand what is going on
    # Second bit marks menu button
    controller_state['menu_button'] = bool(pControllerState.ulButtonPressed >> 1 & 1)
    # 32 bit marks trackpad
    controller_state['trackpad_pressed'] = bool(pControllerState.ulButtonPressed >> 32 & 1)
    controller_state['trackpad_touched'] = bool(pControllerState.ulButtonTouched >> 32 & 1)
    # third bit marks grip button
    controller_state['grip_button'] = bool(pControllerState.ulButtonPressed >> 2 & 1)
    # System button can't be read, if you press it
    # the controllers stop reporting
    return controller_state



class VrPublisher(Node):

    def __init__(self, openvr_system:openvr.IVRSystem):
        super().__init__('vr_publisher')
        timer_period = 0.05
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.devices = {}
        self.system = openvr_system
        self.poses = []
        self.publishers_dict = {}
        self.prev_time = datetime.now()
        self.point = {}
        self.rot = {}
        self.vel = defaultdict(lambda: np.zeros((5,3)))
        self.ang_vel = defaultdict(lambda: np.zeros((5,3)))
        self.hmd_pose = Pose()
        self.last_hmd_pose = None
        self.base_rot = defaultdict(lambda: pyq.Quaternion())
        self.base_pose = defaultdict(lambda: [0]*3)
        self.controller_state = {}
        self.offset_rot = pyq.Quaternion(w = 0.0, x = 1.0, y = 0.0, z = 0.0)

    def extract_pose(self, tracked_device:openvr.TrackedDevicePose_t, name:str) -> Tuple[Pose, Twist]:
        point = Point()
        velocity = Vector3()
        ang_velocity = Vector3()
        pose = convert_to_quaternion(tracked_device.mDeviceToAbsoluteTracking)
        orig_point = Point()
        orig_point.x = pose[0][0]
        orig_point.y = pose[0][1]
        orig_point.z = pose[0][2]

        point.x = pose[0][0] - self.hmd_pose.position.x
        point.y = pose[0][1] - self.hmd_pose.position.y
        point.z = pose[0][2] - self.hmd_pose.position.z

        # if self.point.get(name, None) is not None:
        self.vel[name] = np.roll(self.vel[name],1,axis=0)
        self.vel[name][-1] = [round(num,2) for num in tracked_device.vVelocity[0:3]]
        self.ang_vel[name] = np.roll(self.ang_vel[name],1,axis=0)
        self.ang_vel[name][-1] = [round(num,2) for num in tracked_device.vAngularVelocity[0:3]]
        mean = np.mean(self.vel[name],0)
        ang_mean = np.mean(self.ang_vel[name],0)
        # velocity.x = round(mean[2],2)+0.0
        # velocity.y = round(mean[1],2)+0.0
        # velocity.z = round(mean[0],2)+0.0
        velocity.x = -mean[2] + 0.0
        velocity.y = mean[1] + 0.0
        velocity.z = -mean[1] + 0.0
        ang_velocity.x = ang_mean[2] + 0.0
        ang_velocity.y = ang_mean[1] + 0.0
        ang_velocity.z = ang_mean[0] + 0.0
        # self.point[name] = point

        rot = Quaternion()
        if self.controller_state.get(name, None) is not None:
            if self.controller_state[name]["trackpad_pressed"]:
                self.base_rot[name] = pyq.Quaternion(pose[1])
            if self.controller_state[name]["ulButtonPressed"] == 6:
                self.base_pose[name] = pose[0]

        q1 = pyq.Quaternion(pose[1])

        rot.w = pose[1][0]
        rot.x = pose[1][1]
        rot.y = pose[1][2]
        rot.z = pose[1][3]


        if name == "right_hand":
            # print(convert_to_euler(tracked_device.mDeviceToAbsoluteTracking))
            pose_msg = Pose()
            pose_msg.orientation = rot
            pose_msg.position = point
            self.publish(f"{name}_original", pose_msg, Pose)

        new_quat = self.offset_rot * q1
        rot.w = new_quat.w
        rot.x = new_quat.x
        rot.y = new_quat.y
        rot.z = new_quat.z
        
        # rot.w = q1.w
        # rot.x = q1.x
        # rot.y = q1.y
        # rot.z = q1.z
        
        hmd_msg = Pose()
        if name == "hmd":
            hmd_msg.orientation = rot
            hmd_msg.position = orig_point
            self.hmd_pose = hmd_msg

        self.rot[name] = q1
        pose_msg = Pose()
        pose_msg.orientation = rot
        pose_msg.position = point

        vel_msg = Twist()
        vel_msg.linear = velocity
        vel_msg.angular = ang_velocity

        return pose_msg, vel_msg

    def timer_callback(self):
        self.poses = self.system.getDeviceToAbsoluteTrackingPose(
            openvr.TrackingUniverseStanding, 0, self.poses)

        

        ########
        # # ETrackedDeviceClass = ENUM_TYPE
        # # TrackedDeviceClass_Invalid = ENUM_VALUE_TYPE(0)
        # # TrackedDeviceClass_HMD = ENUM_VALUE_TYPE(1)
        # # TrackedDeviceClass_Controller = ENUM_VALUE_TYPE(2)
        # # TrackedDeviceClass_GenericTracker = ENUM_VALUE_TYPE(3)
        # # TrackedDeviceClass_TrackingReference = ENUM_VALUE_TYPE(4)
        # # TrackedDeviceClass_DisplayRedirect = ENUM_VALUE_TYPE(5)
        # # TrackedDeviceClass_Max = ENUM_VALUE_TYPE(6)
        ########
        # # TrackedControllerRole_LeftHand = 1,					// Tracked device associated with the left hand
        # # TrackedControllerRole_RightHand = 2,				// Tracked device associated with the right hand
        ########
        for idx, controller in enumerate(self.poses):
            # Needed as the order of the devices may change ( based on which thing got turned on first)
            if not controller.bDeviceIsConnected:
                continue
            if self.system.getTrackedDeviceClass(idx) == 1:
                self.devices["hmd"] = (controller, idx)
            elif self.system.getTrackedDeviceClass(idx) == 2 and (self.devices.get("left_hand",None) is None or self.devices.get("right_hand", None) is None):
                controller_role = self.system.getControllerRoleForTrackedDeviceIndex(
                    idx)
                    
                if controller_role == 1:
                    self.devices["left_hand"] = (controller, idx)
                if controller_role == 2:
                    self.devices["right_hand"] = (controller, idx)
            elif self.system.getTrackedDeviceClass(idx) == 3:
                    self.devices[f"tracker_{idx}"]= (controller, idx)

        for name, (el, idx) in self.devices.items():
            if "hand" in name:
                result, pControllerState = self.system.getControllerState(
                    idx)
                if result and name:
                    self.controller_state[name] = from_controller_state_to_dict(pControllerState)
                    # if name == "RightHand":
                    #     print(self.controller_state)
                    tmp_name = f"{name}/trigger"
                    msg = Bool()
                    msg.data = self.controller_state[name]["ulButtonPressed"] >= 8589934592 # At least TRigger pressed
                    self.publish(tmp_name, msg, Bool)
                    

        
        
            pose_msg, vel_msg = self.extract_pose(el, name)
            if self.controller_state[name]["ulButtonPressed"] == 6:
                pose_msg.position.x = 3001 ## we want to disable tracking if the trackpad is pressed

            name = f"{name}/pose"

            self.publish(name, pose_msg, Pose)

            name = name.replace("pose", "vel")
            self.publish(name, vel_msg, Twist)

    def publish(self, name, value, type):
        pub = self.publishers_dict.get(name)
        if pub is None:
            pub = self.create_publisher(type, name, 10)
            self.publishers_dict[name] = pub
        pub.publish(value)


def main(args=None):
    if not openvr.isRuntimeInstalled:
        raise RuntimeError("OpenVR / SteamVr is not Installed Exit")
    if not openvr.isHmdPresent():
        raise RuntimeError(
            "SteamVr is not running or Headmount is not plugged in")

    rclpy.init(args=args)
    system = openvr.init(openvr.VRApplication_Scene)

    minimal_publisher = VrPublisher(system)

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()
    openvr.shutdown()


if __name__ == '__main__':
    main()
