import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
import sys
import sympy as sp
from sympy import symbols, Matrix, pprint
from mycobot320_analysis.robot_model.mycobot320 import MyCobot320, SE3

class WristSingularityAnalyzer(Node):
    def __init__(self):
        super().__init__('wrist_singularity_analyzer')
        self.robot = MyCobot320()
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.joint_names = [f'joint{i+1}' for i in range(6)]
        self.calculate_singularity_pose()
        # Define the wrist singularity configuration (q5 = 0, +-pi)
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
            self.get_logger().info("Jacobian recovered full rank early. Singularity is EXTERNAL or degenerate.")

        if self.traj.size == 0:
            self.get_logger().warn("Jacobian has full rank at this configuration. No null-space motion possible.")

        self.repeat_limit = 10
        self.repeat_counter = 0
        self.traj_index = 0

        self.phase = 'nullspace'
        # Start publishing to /joint_states
        self.timer = self.create_timer(0.2, self.timer_callback)

    def calculate_singularity_pose(self):
        """
        Computes the full symbolic Jacobian of the manipulator using spatial velocity 
        projections, then extracts the sub-Jacobian J22 for the wrist and analyzes 
        the singularity condition based on its determinant.

        Steps:
        ------
        1. Each Jacobian column is constructed using the velocity propagation rule,
        projecting angular and linear velocities through the homogeneous transformation matrices U1 to U6.

        2. Once the full Jacobian is built symbolically, the last 3 columns correspond 
        to the wrist (spherical structure). The submatrix J22 is defined symbolically 
        and analyzed.

        3. The wrist singularity occurs when the determinant of J22 becomes zero,
        which is found analytically.

        This function serves as both a constructive and analytical tool to understand 
        how singularities emerge from the Jacobian structure.
        """

        # Build full Jacobian symbolically
        # Each Jacobian column is constructed using the velocity propagation rule, 
        # projecting angular and linear velocities through the homogeneous transformation matrices U1 to U6.
        c6, s6, d6, c5, s5, d5 = symbols('c6 s6 d6 c5 s5 d5')
        c4, s4, d4, s3, c3, s2, c2, s1, c1, d1, a2, a3 = symbols('c4 s4 d4 s3 c3 s2 c2 s1 c1 d1 a2 a3')
        U521, U522 = symbols('U521 U522')
        U411, U412, U413, U414, U421, U422, U423, U424, U431, U432, U433, U434 = symbols('U411 U412 U413 U414 U421 U422 U423 U424 U431 U432 U433 U434')
        U311, U312, U313, U314, U321, U322, U323, U324, U331, U332, U333, U334 = symbols('U311 U312 U313 U314 U321 U322 U323 U324 U331 U332 U333 U334')
        U211, U212, U213, U214, U221, U222, U223, U224, U231, U232, U233, U234 = symbols('U211 U212 U213 U214 U221 U222 U223 U224 U231 U232 U233 U234')
        U111, U112, U113, U114, U121, U122, U123, U124, U131, U132, U133, U134 = symbols('U111 U112 U113 U114 U121 U122 U123 U124 U131 U132 U133 U134')

        U6 = Matrix([
            [c6, -s6, 0, 0],
            [s6,  c6, 0, 0],
            [0,    0, 1, d6],
            [0,    0, 0, 1]
        ])

        U5 = Matrix([
            [c5 * s6, -c5 * s6, -s5, -d6 * s5],
            [c6 * s5, -s5 * s6,  c5,  c5 * d6],
            [-s6,     -c6,        0,  d5],
            [0,        0,         0,  1]
        ])

        U4 = Matrix([
            [U411, U412, s4 * s5, U414],
            [U421, U422, -c4 * s5, U424],
            [U521, U522,     c5,   U434],
            [0,     0,        0,     1]
        ])

        U3 = Matrix([
            [U311, U312, U313, U314],
            [U321, U322, U323, U324],
            [U521, U522,   c5, U434],
            [0,     0,      0,   1]
        ])

        U2 = Matrix([
            [U211, U212, U213, U214],
            [U221, U222, U223, U224],
            [U521, U522,   c5, U434],
            [0,     0,      0,   1]
        ])

        U1 = Matrix([
            [U111, U112, U113, U114],
            [U121, U122, U123, U124],
            [-U221, -U222, U223, U134],
            [0,     0,     0,     1]
        ])

        # Define joint axes and linear velocity terms (symbolic)
        J1a6 = self.project_spatial_velocity_to_frame(U2, Matrix([0, -1, 0]), Matrix([0, 0, 0]))
        J2a6 = self.project_spatial_velocity_to_frame(U3, Matrix([0,  0, 0]), Matrix([0, a2, 0]))
        J3a6 = self.project_spatial_velocity_to_frame(U4, Matrix([0, -1, 0]), Matrix([0, a3, 0]))
        J4a6 = self.project_spatial_velocity_to_frame(U5, Matrix([0,  1, 0]), Matrix([0, 0, 0]))
        J5a6 = self.project_spatial_velocity_to_frame(U6, Matrix([0, -1, 0]), Matrix([0, 0, 0]))
        J6a6 = self.project_spatial_velocity_to_frame(Matrix.eye(4), Matrix([0, 0, 1]), Matrix([0, 0, 0]))

        # Combine to full Jacobian
        J = J1a6.row_join(J2a6).row_join(J3a6).row_join(J4a6).row_join(J5a6).row_join(J6a6)

        self.get_logger().info("Full symbolic Jacobian J:")
        pprint(J)

        # Step 2: Isolate and analyze J22 sub-Jacobian
        q5, q6 = symbols('q5 q6')
        J22 = Matrix([
            [sp.cos(q6) * sp.sin(q5), -sp.sin(q6), 0],
            [-sp.sin(q5) * sp.sin(q6), -sp.cos(q6), 0],
            [sp.cos(q5), 0, 1]
        ])

        self.get_logger().info("Wrist sub-Jacobian J22:")
        pprint(J22)

        detJ22 = J22.det().simplify()
        self.get_logger().info(f"Determinant of J22: {detJ22}")

        singular_q5 = sp.solve(detJ22, q5)
        self.get_logger().info(f"Wrist singularities occur when q5 = {singular_q5}")


    @staticmethod
    def project_spatial_velocity_to_frame(T, w, v):
        """
        Projects spatial velocity (linear and angular) expressed in a previous frame
        onto a new frame defined by a homogeneous transformation matrix T.

        This implements Equation (75) from the analytical Jacobian derivation:
        tau_vx = n · (ω x p + v)
        tau_vy = o · (ω x p + v)
        tau_vz = a · (ω x p + v)
        tau_ωx = n · ω
        tau_ωy = o · ω
        tau_ωz = a · ω

        """
        n = T[0:3, 0]  # X axis of new frame
        o = T[0:3, 1]  # Y axis of new frame
        a = T[0:3, 2]  # Z axis of new frame
        p = T[0:3, 3]  # Origin of new frame

        w_cross_p = w.cross(p)

        tau = Matrix([
            n.dot(w_cross_p + v),
            o.dot(w_cross_p + v),
            a.dot(w_cross_p + v),
            n.dot(w),
            o.dot(w),
            a.dot(w)
        ])
        return tau

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
                [0,  0, 1, 0.054],
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
        for pose in self.robot.cartesian_move(TA, TB, 20):
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
                    rclpy.end()

        elif self.phase == 'cartesian':
            if self.traj_index < len(self.traj):
                self.publish_joint_state(self.traj[self.traj_index])
                self.traj_index += 1
            else:
                self.get_logger().info("Cartesian trajectory complete.")
                rclpy.end()

    def publish_joint_state(self, q):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = q.tolist()
        self.publisher.publish(msg)

    def end(self):
        self.get_logger().info("WristSingularityAnalyzer node shutting down.")
        self.timer.cancel()
        self.destroy_node() 
        sys.exit(0)       

def main(args=None):
    rclpy.init(args=args)
    node = WristSingularityAnalyzer()
    rclpy.spin(node)
    node.end()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
