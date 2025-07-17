import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
import numpy as np
import itertools
from mycobot320_analysis.utils import create_robot, pose_to_msg

class ShowConfigurationsNode(Node):
    def __init__(self):
        super().__init__('show_configurations_node')

        # Publishers
        self.pose_pub = self.create_publisher(PoseStamped, 'demo_pose', 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)

        # Robot model
        self.robot = create_robot()
        self.joint_names = [
            'joint1',
            'joint2',
            'joint3',
            'joint4',
            'joint5',
            'joint6'
        ]

        # Reference pose
        self.q_base = np.array([-0.15,  0.76,  1.78,  2.47,  0.77, -2.12])

        self.pose = self.robot.fkine_all(self.q_base)[-1]

        # All 8 configuration combinations
        self.configs = list(itertools.product([1, -1], repeat=3))
        self.index = 0

        # Timer to publish each config step by step
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info("ShowConfigurationsNode started.")
        self.repeat_counter = 0
        self.repeat_limit = 5

        # Publish initial JointState 
        q_init = self.q_base
        self.last_q = q_init
        self.publish_joint_state(q_init) 

    def timer_callback(self):
        if self.repeat_counter < self.repeat_limit:
            if hasattr(self, 'last_q'):
                self.publish_joint_state(self.last_q)
            self.repeat_counter += 1
            return

        if self.index >= len(self.configs):
            self.get_logger().info("All configuration combinations processed.")
            rclpy.shutdown()
            return

        config = list(self.configs[self.index])
        q, status = self.robot.ikine_a(self.pose, config)


        if status == 1:
            self.last_q = q
            self.publish_joint_state(q)
            pose_msg = pose_to_msg(self.pose)
            self.pose_pub.publish(pose_msg)
            self.get_logger().info(f"Published config {config}")
        else:
            self.get_logger().warn(f"Configuration {config} is not solvable.")

        self.repeat_counter = 0
        self.index += 1

    def publish_joint_state(self, q):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = self.joint_names
        js.position = list(q)
        #self.get_logger().info(f"Publicando JointState q = {np.round(q, 3)}")
        self.joint_pub.publish(js)    

def main(args=None):
    rclpy.init(args=args)
    node = ShowConfigurationsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()  