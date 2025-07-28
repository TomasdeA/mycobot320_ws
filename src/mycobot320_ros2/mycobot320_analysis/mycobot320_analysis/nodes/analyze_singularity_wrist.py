import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
from mycobot320_analysis.robot_model.mycobot320 import MyCobot320, SE3

class WristSingularityAnalyzer(Node):
    def __init__(self):
        super().__init__('wrist_singularity_analyzer')
        self.robot = MyCobot320()
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.joint_names = [f'joint{i+1}' for i in range(6)]

        # Define the wrist singularity configuration (q5 = 0)
        self.q_sing = np.array([0, 0, np.pi/2, 0, 0, 0])

        # Compute Jacobian and its nullspaces
        J = self.robot.get_jacobian(self.q_sing)
        null_J = self.robot.nullspace(J)
        null_Jt = self.robot.nullspace(J.T)

        self.get_logger().info(f"Null space of J:\n{null_J}")
        self.get_logger().info(f"Null space of Jᵗ:\n{null_Jt}")

        # Generate nullspace trajectory
        self.traj, is_internal = self.robot.trajectory_along_nullspace(self.q_sing, steps=100, step_size=0.01)

        if is_internal:
            self.get_logger().info("Trajectory completed. Singularity is INTERNAL.")
        else:
            self.get_logger().warn("Jacobian recovered full rank early. Singularity is EXTERNAL or degenerate.")

        if self.traj.size == 0:
            self.get_logger().warn("Jacobian has full rank at this configuration. No null-space motion possible.")

        self.repeat_limit = 10
        self.repeat_counter = 0
        self.traj_index = 0

        self.phase = 'nullspace'
        # Start publishing to /joint_states
        self.timer = self.create_timer(0.2, self.timer_callback)

    def generate_cartesian_trajectory(self):
        """
        Generate a Cartesian trajectory across the wrist singularity (along Y axis),
        and convert it to joint-space trajectory using IK.
        """
        # Perturb q5 slightly to move away from exact singularity
        q_around = self.q_sing + np.array([0, 0, 0, 0, 0.1, 0])
        conf = self.robot.calculate_config(q_around)

        # Hardcoded pose frames near the wrist singularity
        TA = SE3([[0, -1, 0, 0.215],
                [0,  0, 1, 0.104],
                [-1, 0, 0, 0.3154],
                [0,  0, 0, 1]])
        
        TB = SE3([[0, -1, 0, 0.215],
                [0,  0, 1, 0.204],
                [-1, 0, 0, 0.3154],
                [0,  0, 0, 1]])

        # Fixed orientation matrix (due to ctraj limitations)
        R_fixed = np.array([[0, -1,     0],
                            [0.09983,  0,  0.995],
                            [-0.995,   0,  0.09983]])

        joint_traj = []
        for pose in self.robot.cartesian_move(TA, TB, 100):
            pose.A[:3, :3] = R_fixed  # Overwrite orientation
            q_sol, status = self.robot.ikine_a(pose, conf)
            if status == 1:
                joint_traj.append(q_sol)

        if joint_traj:
            self.get_logger().info("Cartesian trajectory near wrist singularity generated successfully.")
            self.traj = np.array(joint_traj)
            self.traj_index = 0
            return True
        else:
            self.get_logger().warn("No valid IK solutions for the Cartesian path.")
            return False

    def timer_callback(self):
        if self.phase == 'nullspace':
            if self.repeat_counter < self.repeat_limit:
                self.publish_joint_state(self.q_sing)
                self.repeat_counter += 1
            elif self.traj_index < len(self.traj):
                self.publish_joint_state(self.traj[self.traj_index])
                self.traj_index += 1
            else:
                self.get_logger().info("Null-space trajectory complete. Starting Cartesian trajectory.")
                if self.generate_cartesian_trajectory():
                    self.phase = 'cartesian'
                else:
                    rclpy.shutdown()

        elif self.phase == 'cartesian':
            if self.traj_index < len(self.traj):
                self.publish_joint_state(self.traj[self.traj_index])
                self.traj_index += 1
            else:
                self.get_logger().info("Cartesian trajectory complete.")
                rclpy.shutdown()

    def publish_joint_state(self, q):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = q.tolist()
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = WristSingularityAnalyzer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
