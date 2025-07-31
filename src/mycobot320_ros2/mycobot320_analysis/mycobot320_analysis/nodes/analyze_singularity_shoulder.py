import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
import sys
from mycobot320_analysis.robot_model.mycobot320 import MyCobot320, SE3

class ShoulderSingularityAnalyzer(Node):
    def __init__(self):
        super().__init__('shoulder_singularity_analyzer')
        self.robot = MyCobot320()
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.joint_names = [f'joint{i+1}' for i in range(6)]

        # Extract DH parameters used in inverse kinematics
        d1 = self.robot.links[0].d
        a2 = self.robot.links[1].a
        a3 = self.robot.links[2].a
        d4 = self.robot.links[3].d
        d5 = self.robot.links[4].d
        d6 = self.robot.links[5].d
        # Define a pose that causes the shoulder singularity
        pose_sing = SE3([
            [1, 0, 0, 0],
            [0, 1, 0, d4],
            [0, 0, 1, d1 + a2 + a3 ],
            [0, 0, 0, 1]
        ])
        # Note: the Z value (d1 + a2 + a3) is arbitrary — it places the end-effector at a reachable height

        q_sing, status = self.robot.ikine_a(pose_sing)

        if status != 1:
            self.get_logger().error("Failed to compute IK at shoulder singularity.")
            rclpy.end()
            return

        self.q_sing = q_sing
        self.repeat_limit = 10
        self.repeat_counter = 0
        self.traj_index = 0

        # Compute Jacobian and null spaces using robot methods
        J = self.robot.get_jacobian(self.q_sing)
        null_J = self.robot.nullspace(J)
        null_Jt = self.robot.nullspace(J.T)

        self.get_logger().info(f"Null space of J:\n{null_J}")
        self.get_logger().info(f"Null space of Jᵗ:\n{null_Jt}")

        # Use the robot method to generate trajectory along the nullspace
        self.traj, is_internal = self.robot.trajectory_along_nullspace(self.q_sing, steps=100, step_size=0.01)

        if is_internal:
            self.get_logger().info("Trajectory completed. Singularity is INTERNAL.")
        else:
            self.get_logger().info("Jacobian recovered full rank early. Singularity is EXTERNAL or degenerate.")
        
        if self.traj.size == 0:
            self.get_logger().warn("Jacobian has full rank at this configuration. No null-space motion possible.")

        self.phase = 'nullspace'
        # Start publishing to /joint_states
        self.timer = self.create_timer(0.2, self.timer_callback)

    def generate_cartesian_trajectory(self):
        """
        Generate a Cartesian trajectory across the shoulder singularity (along X axis),
        and convert it to joint-space trajectory using IK.
        """
        d4 = self.robot.links[3].d
        y = d4 + 0.001  # Slightly outside the singularity circle to avoid exact alignment
        z = self.robot.links[0].d + self.robot.links[1].a + self.robot.links[2].a

        TA = SE3(0.10, y, z)
        TB = SE3(-0.10, y, z)

        joint_traj = []

        for pose in self.robot.cartesian_move(TA, TB, 100):
            q_sol, status = self.robot.ikine_a(pose)
            if status == 1:
                joint_traj.append(q_sol)

        if joint_traj:
            self.get_logger().info("Cartesian trajectory near shoulder singularity generated successfully.")
            self.traj = np.array(joint_traj)
            self.traj_index = 0
            return True
        else:
            self.get_logger().warn("No valid IK solutions for the Cartesian path.")
            return False

    def timer_callback(self):
        # First publish singular configuration a few times
        if self.repeat_counter < self.repeat_limit:
            self.publish_joint_state(self.q_sing)
            self.repeat_counter += 1
        # Then publish the null-space trajectory
        elif self.traj_index < len(self.traj):
            self.publish_joint_state(self.traj[self.traj_index])
            self.traj_index += 1
        # When null-space motion ends, switch to Cartesian trajectory
        elif hasattr(self, 'phase') and self.phase == 'cartesian':
            self.get_logger().info("Cartesian trajectory complete.")
            rclpy.end()
        else:
            self.get_logger().info("Null-space trajectory complete. Starting Cartesian trajectory.")
            if self.generate_cartesian_trajectory():
                self.phase = 'cartesian'
            else:
                rclpy.end()

    def publish_joint_state(self, q):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = q.tolist()
        self.publisher.publish(msg)

    def end(self):
        self.get_logger().info("ShoulderSingularityAnalyzer node shutting down.")
        self.timer.cancel()
        self.destroy_node() 
        sys.exit(0)       


def main(args=None):
    rclpy.init(args=args)
    node = ShoulderSingularityAnalyzer()
    rclpy.spin(node)
    node.end()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
