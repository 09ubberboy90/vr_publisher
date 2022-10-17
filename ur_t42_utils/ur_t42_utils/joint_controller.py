import time
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import JointState

from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import Float64MultiArray

JOINT_ANGLE_TOLERANCE = 0.008726646
HEAD_PAN_ANGLE_TOLERANCE = 0.1396263401

class JointController(Node):

    def __init__(self):
        super().__init__('baxter_joint_controller')
        self.publisher = self.create_publisher(Float64MultiArray, '/forward_position_controller/commands', 10)

        self.subscription = self.create_subscription(
            JointState,
            'joint_states_sim',
            self.listener_callback,
            10)

        self.joint_states = {}
        self.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]

    def listener_callback(self, msg:JointState):
        for name, pose in zip(msg.name, msg.position):
            self.joint_states[name] = pose
        
        out_msg = Float64MultiArray()
         
        for el in self.joint_names:
            out_msg.data.append(self.joint_states[el])
        
        self.publisher.publish(out_msg)



def main(args=None):
    rclpy.init(args=args)

    node = JointController()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
