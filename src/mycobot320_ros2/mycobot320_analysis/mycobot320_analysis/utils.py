import numpy as np
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as R
import rclpy

class MyCobot320:
    def __init__(self):
        # Parámetros DH reales
        self.d1 = 0.1739
        self.a2 = 0.135
        self.a3 = 0.120
        self.d4 = 0.08878
        self.d5 = 0.095
        self.d6 = 0.0655
        self.dh_params = [
            [-np.pi/2, 0,     self.d1,     0],
            [0,        self.a2, 0,         -np.pi/2],
            [0,        self.a3, 0,         0],
            [np.pi/2,  0,     self.d4,     np.pi/2],
            [-np.pi/2, 0,     self.d5,     0],
            [0,        0,     self.d6,     0],
        ]

    def A(self, alpha, a, d, theta):
        sa, ca = np.sin(alpha), np.cos(alpha)
        st, ct = np.sin(theta), np.cos(theta)
        return np.array([
            [ct, -st * ca,  st * sa, a * ct],
            [st,  ct * ca, -ct * sa, a * st],
            [0,       sa,       ca,     d],
            [0,        0,        0,     1]
        ])

    def fkine_all(self, q):
        T = np.eye(4)
        Ts = [T.copy()]
        for i in range(6):
            alpha, a, d, offset = self.dh_params[i]
            T_i = self.A(alpha, a, d, q[i] + offset)
            T = T @ T_i
            Ts.append(T.copy())
        return Ts

    def fkine(self, q):
        return self.fkine_all(q)[-1]

    def calc_conf(self, q):
        Ts = self.fkine_all(q)
        p1 = np.linalg.inv(Ts[1]) @ Ts[6]
        p16_x = p1[0, 3]
        return np.array([-np.sign(p16_x), np.sign(q[2]), np.sign(q[4])], dtype=int)

    def ikine_a(self, Ts, conf=[1, 1, 1]):
        if isinstance(Ts, list):
            T = Ts[6]
        else:
            T = Ts
        px, py, pz = T[0:3, 3]
        nx, ny, nz = T[0:3, 0]
        sx, sy, sz = T[0:3, 1]
        ax, ay, az = T[0:3, 2]

        d1, a2, a3, d4, d5, d6 = self.d1, self.a2, self.a3, self.d4, self.d5, self.d6
        conf1, conf2, conf3 = conf

        discr = (px - ax*d6)**2 + (py - ay*d6)**2 - d4**2
        if discr < 0:
            return [], -1

        q1 = np.arctan2(d4, conf1 * np.sqrt(discr)) - np.arctan2(py - ay*d6, ax*d6 - px)
        q5 = np.arctan2(conf3*np.sqrt((ny*np.cos(q1) - nx*np.sin(q1))**2 + (sy*np.cos(q1) - sx*np.sin(q1))**2),
                        ay*np.cos(q1) - ax*np.sin(q1))
        q6 = np.arctan2(-conf3*(sy*np.cos(q1) - sx*np.sin(q1)),
                         conf3*(ny*np.cos(q1) - nx*np.sin(q1)))
        theta234 = np.arctan2(az*conf3, -conf3*(ax*np.cos(q1) + ay*np.sin(q1)))

        A = px*np.cos(q1) - d5*np.sin(theta234) + py*np.sin(q1) + d6*np.cos(theta234)*np.sin(q5)
        B = d1 - pz + d5*np.cos(theta234) + d6*np.sin(q5)*np.sin(theta234)

        c3 = (A**2 + B**2 - a2**2 - a3**2) / (2*a2*a3)
        if abs(c3) > 1:
            return [], -1

        q3 = np.arctan2(conf2*np.sqrt(1 - c3**2), c3)
        s2 = (B*a2 + B*a3*np.cos(q3) - A*a3*np.sin(q3)) / (a2**2 + a3**2 + 2*a2*a3*np.cos(q3))
        c2 = (A*a2 + A*a3*np.cos(q3) + B*a3*np.sin(q3)) / (a2**2 + a3**2 + 2*a2*a3*np.cos(q3))
        q2 = np.arctan2(s2, c2)
        q4 = theta234 - q2 - q3

        q = np.array([q1, q2, q3, q4, q5, q6])
        q = (q + np.pi) % (2 * np.pi) - np.pi
        return q, 1

def create_robot():
    return MyCobot320()

def pose_to_msg(T):
    msg = PoseStamped()
    msg.header.frame_id = "base_link"
    msg.header.stamp = rclpy.clock.Clock().now().to_msg()
    msg.pose.position.x = T[0, 3]
    msg.pose.position.y = T[1, 3]
    msg.pose.position.z = T[2, 3]

    rot = R.from_matrix(T[0:3, 0:3])
    q = rot.as_quat()
    msg.pose.orientation.x = q[0]
    msg.pose.orientation.y = q[1]
    msg.pose.orientation.z = q[2]
    msg.pose.orientation.w = q[3]
    return msg