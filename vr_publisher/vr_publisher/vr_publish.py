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

import sys
from datetime import datetime
import openvr
import pyquaternion as pyq
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, Vector3
from std_msgs.msg import Bool
from collections import defaultdict

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
    r_w = math.sqrt(abs(1+pose_mat[0][0]+pose_mat[1][1]+pose_mat[2][2]))/2
    r_x = (pose_mat[2][1]-pose_mat[1][2])/(4*r_w)
    r_y = (pose_mat[0][2]-pose_mat[2][0])/(4*r_w)
    r_z = (pose_mat[1][0]-pose_mat[0][1])/(4*r_w)
    
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
    d = {}
    d['unPacketNum'] = pControllerState.unPacketNum
    # on trigger .y is always 0.0 says the docs
    d['trigger'] = pControllerState.rAxis[3].x
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
    d['trackpad_x'] = pControllerState.rAxis[0].x
    d['trackpad_y'] = pControllerState.rAxis[0].y
    # These are published and always 0.0
    # for i in range(2, 5):
    #     d['unknowns_' + str(i) + '_x'] = pControllerState.rAxis[i].x
    #     d['unknowns_' + str(i) + '_y'] = pControllerState.rAxis[i].y
    d['ulButtonPressed'] = pControllerState.ulButtonPressed
    d['ulButtonTouched'] = pControllerState.ulButtonTouched
    # To make easier to understand what is going on
    # Second bit marks menu button
    d['menu_button'] = bool(pControllerState.ulButtonPressed >> 1 & 1)
    # 32 bit marks trackpad
    d['trackpad_pressed'] = bool(pControllerState.ulButtonPressed >> 32 & 1)
    d['trackpad_touched'] = bool(pControllerState.ulButtonTouched >> 32 & 1)
    # third bit marks grip button
    d['grip_button'] = bool(pControllerState.ulButtonPressed >> 2 & 1)
    # System button can't be read, if you press it
    # the controllers stop reporting
    return d


class VrPublisher(Node):

    def __init__(self, openvr_system, buttons=False, raw=False):
        super().__init__('vr_publisher')
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.devices = defaultdict(list)
        self.system = openvr_system
        self.poses = []
        self.publishers_dict = {}
        self.prev_time = datetime.now()
        self.point = {}
        self.velocity = Vector3()
        self.ang_velocity = Vector3()
        self.rot = {}
        self.buttons = buttons
        self.raw = raw
        self.hmd_pose = Pose()
        # self.hmd_pose.position = Point()
        # self.hmd_pose.orientation = Quaternion()

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
            if not self.system.isTrackedDeviceConnected(idx):
                continue
            if self.system.getTrackedDeviceClass(idx) == 1 and len(self.devices["hmd"]) < 1:
                self.devices["hmd"].append(("hmd", controller))
            elif self.system.getTrackedDeviceClass(idx) == 2 and len(self.devices["controller"]) < 2:
                controller_role = self.system.getControllerRoleForTrackedDeviceIndex(
                    idx)
                print(f"Controller {controller_role}")
                hand = ""
                if (controller_role == 1):
                    hand = "LeftHand"
                if controller_role == 2:
                    hand = "RightHand"
                self.devices["controller"].append((hand, controller))
        for key, device in self.devices.items():
            for idx, (name, el) in enumerate(device):
                if key == "controller":
                    result, pControllerState = self.system.getControllerState(
                        idx)
                    if result and name:
                        d = from_controller_state_to_dict(pControllerState)
                        tmp_name = f"{key}/{name}/trigger"
                        msg = Bool()
                        msg.data = d["trigger"] > 0.0
                        self.publish(tmp_name, msg, Bool)

                pose = convert_to_quaternion(
                    el.mDeviceToAbsoluteTracking)
                orig_point = Point()
                point = Point()
                orig_point.x = pose[0][0]
                orig_point.y = pose[0][1]
                orig_point.z = pose[0][2]

                point.x = pose[0][0] - self.hmd_pose.position.x
                point.y = pose[0][1] - self.hmd_pose.position.y
                point.z = pose[0][2] - self.hmd_pose.position.z
                time = datetime.now()
                dtime = (time-self.prev_time).total_seconds()
                self.prev_time = time

                if self.point.get(name, None) is not None:
                    self.velocity.x = round(point.x - self.point[name].x,4) / dtime
                    self.velocity.y = round(point.y - self.point[name].y,4) / dtime
                    self.velocity.z = round(point.z - self.point[name].z,4) / dtime
                self.point[name] = point

                rot = Quaternion()

                q1 = pyq.Quaternion(pose[1])
                q1 = q1.normalised

                rot.w = pose[1][0]
                rot.x = pose[1][1]
                rot.y = pose[1][2]
                rot.z = pose[1][3]
                
                hmd_msg = Pose()
                if name == "hmd":
                    hmd_msg.orientation = rot
                    hmd_msg.position = orig_point
                    self.hmd_pose = hmd_msg

                if self.rot.get(name, None) is not None:
                    diffQuater = q1 - self.rot[name]
                    conjBoxQuater = q1.inverse
                    velQuater = ((diffQuater * 2.0) / dtime) * conjBoxQuater
                    self.ang_velocity.x = velQuater[1]
                    self.ang_velocity.y = velQuater[2]
                    self.ang_velocity.z = velQuater[3]
                    # print(self.ang_velocity)
                self.rot[name] = q1

                msg = Pose()
                msg.orientation = rot
                msg.position = point

                name = f"{key}/{name}"

                self.publish(name, msg, Pose)

                vel = Twist()
                if self.velocity.x == 0.0 and self.velocity.y == 0.0 and self.velocity.z == 0.0 and \
                        self.ang_velocity.x == 0.0 and self.ang_velocity.y == 0.0 and self.ang_velocity.z == 0.0:
                    continue
                vel.linear = self.velocity
                vel.angular = self.ang_velocity

                name += "/vel"
                self.publish(name, vel, Twist)

    def publish(self, name, value, type):
        pub = self.publishers_dict.get(name)
        if pub is None:
            pub = self.create_publisher(type, name, 10)
            self.publishers_dict[name] = pub
        pub.publish(value)


def main(buttons=False, args=None):
    if not openvr.isRuntimeInstalled:
        raise RuntimeError("OpenVR / SteamVr is not Installed Exit")
    if not openvr.isHmdPresent():
        raise RuntimeError(
            "SteamVr is not running or Headmount is not plugged in")

    rclpy.init(args=args)
    system = openvr.init(openvr.VRApplication_Scene)

    minimal_publisher = VrPublisher(system, buttons)

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()
    openvr.shutdown()


if __name__ == '__main__':
    main()
