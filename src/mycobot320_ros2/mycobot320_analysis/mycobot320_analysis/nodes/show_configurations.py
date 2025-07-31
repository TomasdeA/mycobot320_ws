import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
import itertools
import sys
from mycobot320_analysis.robot_model.mycobot320 import MyCobot320

class ShowConfigurationsNode(Node):
    def __init__(self):
        super().__init__('show_configurations_node')
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.robot = MyCobot320()
        self.joint_names = [f'joint{i+1}' for i in range(6)]

        # Reference pose reachable by all 8 configurations

        #self.q_base = np.array([1.614,  2.089,  -1.78,  -1.443,  0.017, -0.866])  # to many collisions, but "reachable" by all 8 configurations
        #self.q_base = np.array([ 0.1,  0.8, -1.2, -1.0, -0.4, 0.2]) # example with unreachable configurations
        #self.q_base = np.array([1, -0.934, 2.496, -2.564, 0, 0]) # to many collisions, but "reachable" by all 8 configurations
        #self.q_base = np.array([0, -0.02359878, 1, 1.52359878, 0.5, 0]) # the best one yet
        self.q_base = np.array([0, 0.5, 1., 1.52359878, -0.02359878, 0])
        
        self.pose_list = self.robot.fkine_all(self.q_base)
        self.pose = self.pose_list[-1]

        self.configs = list(itertools.product([1, -1], repeat=3))
        self.index = 0
        self.repeat_counter = 0
        self.repeat_limit = 5

        self.last_q = np.zeros(6)
        self.publish_joint_state(self.last_q )
        self.timer = self.create_timer(1.0, self.timer_callback)

        self.traj_path = []
        self.traj_index = 0
        self.traj_timer = self.create_timer(0.05, self.publish_trajectory_step)
        self.traj_goal_reached = True

        self.get_logger().info("ShowConfigurationsNode started.")

    def timer_callback(self):
        # Wait until the last reached q is published enough times
        if self.traj_goal_reached:
            if self.repeat_counter < self.repeat_limit:
                self.publish_joint_state(self.last_q)
                self.repeat_counter += 1
                return

            # Done repeating the pose, move to the next configuration
            if self.index >= len(self.configs):
                self.get_logger().info("Finished all 8 configurations.")
                rclpy.end()
                return

            config = list(self.configs[self.index])
            q, status = self.robot.ikine_a(self.pose, config)

            if status == 1:
                config_calc = self.robot.calculate_config(q).tolist()
                q_path = self.robot.joint_move(self.last_q, q, steps=50)

                self.traj_goal_reached = False
                self.traj_index = 0
                self.traj_path = q_path

                self.get_logger().info(f"desired_config={config} | resulting_config={config_calc}")
            else:
                self.get_logger().warn(f"desired_config={config} → Not reachable")

            self.repeat_counter = 0
            self.index += 1

    def publish_joint_state(self, q):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = self.joint_names
        js.position = q.tolist()
        self.publisher.publish(js)

    def publish_trajectory_step(self):
        if self.traj_index < len(self.traj_path):
            q = self.traj_path[self.traj_index]
            self.publish_joint_state(q)
            self.traj_index += 1
        elif len(self.traj_path) > 0:
            self.last_q = self.traj_path[-1]  # Update the last reached q
            self.traj_path = []               # Clear trajectory to avoid repetitions
            self.traj_goal_reached = True

    def end(self):
        self.get_logger().info("ShowConfigurationsNode shutting down.")
        self.timer.cancel()
        self.destroy_node() 
        sys.exit(0)       

def main(args=None):
    rclpy.init(args=args)
    node = ShowConfigurationsNode()
    rclpy.spin(node)
    node.end()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
  