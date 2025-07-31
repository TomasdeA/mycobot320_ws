"""
IMPORTANT: You must have display.launch.py from the mycobot320_description package running in another terminal to visualize the robot in RViz while using this script.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
import itertools
import sys
import os
import json
from mycobot320_analysis.robot_model.mycobot320 import MyCobot320

SAVE_PATH = "/tmp/mycobot_explorer_state.json" 

class ExploreConfigurations(Node):
    def __init__(self, resume_from_file):
        super().__init__('explore_configurations')
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.robot = MyCobot320(symbolic=False)
        self.joint_names = [f'joint{i+1}' for i in range(6)]

        self.q_base = np.array([0.0, 0.5, 1.0, 1.0, 0.5, 0.0])
        self.q_delta = np.deg2rad(30)
        self.perturbations = self.generate_perturbations()

        self.configs = list(itertools.product([1, -1], repeat=3))

        if resume_from_file and os.path.exists(SAVE_PATH):
            self.load_progress()
        else:
            self.pose_index = 0
            self.config_index = 0
            self.valid_solutions = []

        self.last_q = self.q_base.copy()
        self.evaluate_next_pose()

    def generate_perturbations(self):
        offsets = [-self.q_delta, 0.0, self.q_delta]
        return list(itertools.product(offsets, offsets, offsets, offsets, offsets, [0.0]))

    def load_progress(self):
        with open(SAVE_PATH, 'r') as f:
            data = json.load(f)
        self.pose_index = data['pose_index']
        self.config_index = data['config_index']
        self.valid_solutions = [(tuple(x[0]), np.array(x[1])) for x in data['valid_solutions']]
        print(f"📂 Progress resumed from {SAVE_PATH}")
        print(f"  → Pose #{self.pose_index}, Config #{self.config_index}")

    def save_progress(self):
        data = {
            'pose_index': self.pose_index,
            'config_index': self.config_index,
            'valid_solutions': [(list(c[0]), c[1].tolist()) for c in self.valid_solutions]
        }
        with open(SAVE_PATH, 'w') as f:
            json.dump(data, f, indent=2)

    def evaluate_next_pose(self):
        if self.pose_index >= len(self.perturbations):
            self.get_logger().info("🏁 Exploration finished.")
            print(f"\n✅ Total valid poses found: {len(self.valid_solutions) // 8}")
            os.remove(SAVE_PATH)
            rclpy.shutdown()
            return

        perturb = np.array(self.perturbations[self.pose_index])
        self.q_test = self.q_base + perturb
        self.pose = self.robot.fkine(self.q_test)

        print("\n🟨 Evaluating POSE derived from perturbed q_base:")
        print(self.q_test)
        print(self.pose)

        self.current_pose_valid = True
        self.solutions_for_this_pose = []
        self.evaluate_next_config()

    def evaluate_next_config(self):
        if self.config_index >= len(self.configs):
            if len(self.solutions_for_this_pose) == 8:
                print("✅ POSE accepted: all 8 configurations are valid.")
                self.valid_solutions.extend(self.solutions_for_this_pose)
            else:
                print("❌ POSE rejected: one or more configurations failed.")
            self.pose_index += 1
            self.config_index = 0
            self.save_progress()
            self.evaluate_next_pose()
            return

        config = list(self.configs[self.config_index])
        q_sol, status = self.robot.ikine_a(self.pose, config)

        if status != 1:
            print(f"❌ desired_config={config} → Not reachable")
            self.config_index += 1
            self.evaluate_next_config()
            return

        config_calc = self.robot.calculate_config(q_sol).tolist()
        self.get_logger().info(f"desired_config={config} | resulting_config={config_calc}")

        self.publish_joint_state(q_sol)
        self.last_q = q_sol
        self.current_config = config
        self.current_q = q_sol

        user_input = input("Collision? [x = without collision / c = collision / q = abort]: ").strip().lower()
        if user_input == 'q':
            print("🟥 Interrupted by user.")
            self.save_progress()
            rclpy.shutdown()
            return
        elif user_input == 'c':
            print(f"❌ desired_config={config} → Collision detected")
            self.config_index += 1
            self.evaluate_next_config()
            return
        elif user_input == 'x':
            self.solutions_for_this_pose.append((config, q_sol))
            self.config_index += 1
            self.save_progress()
            self.evaluate_next_config()

    def publish_joint_state(self, q):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = q.tolist()
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    # Ask the user if they want to resume previous progress
    option = input("\nDo you want to resume previous progress? [r = resume / n = new]: ").strip().lower()
    resume = option == 'r'

    node = ExploreConfigurations(resume)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
