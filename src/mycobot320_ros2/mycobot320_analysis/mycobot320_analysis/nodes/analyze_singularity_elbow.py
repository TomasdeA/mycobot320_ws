import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
import sys
from mycobot320_analysis.robot_model.mycobot320 import MyCobot320, SE3

class ElbowSingularityAnalyzer(Node):
    def __init__(self):
        super().__init__('elbow_singularity_analyzer')
        self.robot = MyCobot320()
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.joint_names = [f'joint{i+1}' for i in range(6)]

        # Joint configuration that causes the elbow singularity (q3 = 0)
        self.q_sing = np.array([0, np.pi/2, 0, 0, -np.pi/2, 0])

        # Compute Jacobian and its nullspaces
        J = self.robot.get_jacobian(self.q_sing)
        null_J = self.robot.nullspace(J)
        null_Jt = self.robot.nullspace(J.T)

        self.get_logger().info(f"Null space of J:\n{null_J}")
        self.get_logger().info(f"Null space of Jᵗ:\n{null_Jt}")

        # Generate a trajectory along the nullspace of the Jacobian
        self.traj, is_internal = self.robot.trajectory_along_nullspace(self.q_sing, steps=100, step_size=0.01)

        # Report if the singularity is internal or external
        if is_internal:
            self.get_logger().info("Trajectory completed. Singularity is INTERNAL.")
        else:
            self.get_logger().info("Jacobian recovered full rank early. Singularity is EXTERNAL or degenerate.")
        if self.traj.size == 0:
            self.get_logger().warn("Jacobian has full rank at this configuration. No null-space motion possible.")

        # Variables to control the publishing of the trajectory
        self.repeat_limit = 10
        self.repeat_counter = 0
        self.traj_index = 0

        self.phase = 'nullspace'  # Current phase: 'nullspace' or 'cartesian'
        # Start periodic publishing to /joint_states
        self.timer = self.create_timer(0.2, self.timer_callback)


    def generate_cartesian_trajectory(self):
        # Generate a Cartesian trajectory near the elbow singularity
        q_around = self.q_sing + np.array([0, 0, 0.05, 0, 0, 0])
        conf = self.robot.calculate_config(q_around)

        # Extract DH parameters used in inverse kinematics
        d1 = self.robot.links[0].d
        a2 = self.robot.links[1].a
        a3 = self.robot.links[2].a
        d4 = self.robot.links[3].d
        d5 = self.robot.links[4].d
        d6 = self.robot.links[5].d
        # Define two SE3 poses for the Cartesian path
        TA = SE3([[0, -1, 0, a2 + a3 + d5],
                  [-1, 0, 0, d4],
                  [0, 0, -1, d1-d6],
                  [0, 0, 0, 1]])
        TB = SE3([[0, -1, 0, a2 + a3 + d5 - 0.1],
                  [-1, 0, 0, d4],
                  [0, 0, -1, d1-d6],
                  [0, 0, 0, 1]])

        # Fixed rotation for the end-effector orientation
        #R_fixed = np.array([[0, -0.9998, -0.04998],
        #                    [-1, 0, 0],
        #                    [0, 0.04998, -0.9998]])

        joint_traj = []
        # Solve IK for each Cartesian pose to build the joint trajectory
        for pose in self.robot.cartesian_move(TA, TB, 20):
            #pose.A[:3, :3] = R_fixed
            q_sol, status = self.robot.ikine_a(pose, conf)
            if status == 1:
                joint_traj.append(q_sol)

        if joint_traj:
            self.get_logger().info("Cartesian trajectory near elbow singularity generated successfully.")
            self.traj = np.array(joint_traj)
            self.traj_index = 0
            return True
        else:
            self.get_logger().warn("No valid IK solutions for the Cartesian path.")
            return False

    def timer_callback(self):
        if self.repeat_counter < self.repeat_limit:
            self.publish_joint_state(self.q_sing)
            self.repeat_counter += 1

        # Publish the nullspace trajectory
        elif self.traj_index < len(self.traj):
            self.publish_joint_state(self.traj[self.traj_index])
            self.traj_index += 1

        # When nullspace trajectory is done, generate and publish the Cartesian trajectory
        elif self.phase == 'nullspace':
            self.get_logger().info("Null-space trajectory complete. Starting Cartesian trajectory.")
            if self.generate_cartesian_trajectory():
                self.phase = 'cartesian'
            else:
                self.end()

        # Publish the Cartesian trajectory
        elif self.traj_index < len(self.traj):
            self.publish_joint_state(self.traj[self.traj_index])
            self.traj_index += 1

        # End the program when all trajectories are done
        else:
            self.get_logger().info("Cartesian trajectory complete.")
            self.end()

    def publish_joint_state(self, q):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = q.tolist()
        self.publisher.publish(msg)

    def end(self):
        self.get_logger().info("ElbowSingularityAnalyzer node shutting down.")
        self.timer.cancel()
        self.destroy_node() 
        sys.exit(0)       

def main(args=None):
    rclpy.init(args=args)
    node = ElbowSingularityAnalyzer()
    rclpy.spin(node)
    node.end()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
