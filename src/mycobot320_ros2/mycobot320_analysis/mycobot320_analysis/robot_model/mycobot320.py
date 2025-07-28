import numpy as np
import scipy as sc
from sympy import symbols, Matrix, pprint
from roboticstoolbox import DHRobot, RevoluteDH
from roboticstoolbox.tools.trajectory import ctraj, jtraj
import matplotlib.pyplot as plt
import spatialmath as sm

SE3 = sm.SE3  # Alias for SE3 from spatialmath

class MyCobot320(DHRobot):

    def __init__(self, symbolic=False, *args, **kwargs):
        if symbolic:
            d1, a2, a3, d4, d5, d6 = symbols('d1 a2 a3 d4 d5 d6')
        else:
            d1, a2, a3, d4, d5, d6 = 0.1739, 0.135, 0.120, 0.08878, 0.095, 0.0655

        ejes = [
            RevoluteDH(alpha=-np.pi/2, a=0,    d=d1, offset=0,         qlim=[-170*np.pi/180,170*np.pi/180]),
            RevoluteDH(alpha=0,       a=a2,   d=0,  offset=-np.pi/2,  qlim=[-120*np.pi/180,120*np.pi/180]),
            RevoluteDH(alpha=0,       a=a3,   d=0,  offset=0,         qlim=[-148*np.pi/180,148*np.pi/180]),
            RevoluteDH(alpha=np.pi/2, a=0,    d=d4, offset=np.pi/2,   qlim=[-120*np.pi/180,135*np.pi/180]),
            RevoluteDH(alpha=-np.pi/2,a=0,    d=d5, offset=0,         qlim=[-169*np.pi/180,169*np.pi/180]),
            RevoluteDH(alpha=0,       a=0,    d=d6, offset=0,         qlim=[-180*np.pi/180,180*np.pi/180]),
        ]

        super().__init__(*args, ejes, name='myCobot320', gravity=[0, 0, -9.8], symbolic=symbolic, **kwargs)

    def calculate_config(self, q):
        if q.ndim == 1:
            q = q[np.newaxis, :]
        result = []
        for q_row in q:
            A = self.fkine_all(q_row)
            p16_x = (np.linalg.inv(A[1]) @ A[6].A)[0, -1]
            result.append([-np.sign(p16_x), np.sign(q_row[2]), np.sign(q_row[4])])
        return np.array(result, dtype=int).flatten() if q.shape[0] == 1 else np.array(result, dtype=int)

    def ikine_a(self, pose, conf=np.array([1,1,1])):
        """
        Analytical inverse kinematics solver for the myCobot320 robot.

        Given a desired end-effector pose (pose), and a configuration code (conf),
        this method computes the corresponding joint values q = [q1, q2, q3, q4, q5, q6].

        Parameters:
        - pose: spatialmath.SE3 or list of 7 SE3s from fkine_all (joint frames)
        - conf: np.array of shape (3,) with values in {+1, -1}, indicating the desired:
                [shoulder config, elbow config, wrist config]

        Returns:
        - q: numpy array of joint values [q1 to q6] if solvable, empty list otherwise
        - status: 1 if success, -1 if pose is unreachable
        """      
        conf1,conf2,conf3 = conf
        
        # Handle both fkine and fkine_all inputs
        pose_aux = self._get_pose_matrix(pose)

        # Extract position and orientation components
        px, py, pz = pose_aux.t                    # Position
        nx, ny, nz = pose_aux.R[:, 0]              # X-axis of end-effector frame
        sx, sy, sz = pose_aux.R[:, 1]              # Y-axis
        ax, ay, az = pose_aux.R[:, 2]              # Z-axis

        # Extract DH parameters used in inverse kinematics
        d1 = self.links[0].d
        a2 = self.links[1].a
        a3 = self.links[2].a
        d4 = self.links[3].d
        d5 = self.links[4].d
        d6 = self.links[5].d

        R_2 = (px - ax*d6)**2 + (py - ay*d6)**2 # R**2
        shoulder_reach_margin = R_2 - d4**2
        # The value 'shoulder_reach_margin' determines whether the wrist center is within the robot's 
        # reachable workspace in the XY plane for a valid shoulder configuration (θ1).

        if shoulder_reach_margin<0:
            # The wrist center is too close to the base axis — inside the unreachable circular area of 
            # radius d4. No real solution for θ1 exists in this case.
            return [],-1
        if shoulder_reach_margin == 0:
            # The wrist center lies exactly on the boundary of the reachable circle (distance = d4). 
            # There is a unique solution for θ1.
            pass

        # The wrist center lies outside the minimum reachable circle, allowing two possible shoulder 
        # configurations (lefty/righty or front/back) for θ1.

        # Solve q1 (shoulder) from projection in XY plane
        q1 = np.arctan2(d4, conf1*np.sqrt(R_2 - d4**2)) - np.arctan2(py - ay*d6, ax*d6 - px) 
        
        # Solve wrist angles (q5 and q6) from orientation components
        q5 = np.arctan2(conf3*np.sqrt((ny*np.cos(q1) - nx*np.sin(q1))**2 + (sy*np.cos(q1) - sx*np.sin(q1))**2), 
                        ay*np.cos(q1) - ax*np.sin(q1))
        q6 = np.arctan2(-conf3*(sy*np.cos(q1) - sx*np.sin(q1)), 
                        conf3*(ny*np.cos(q1) - nx*np.sin(q1)))
        
        # Compute composite rotation angle of joints 2+3+4
        theta234 = np.arctan2(az*conf3, -conf3*(ax*np.cos(q1) + ay*np.sin(q1)))

        # Geometric terms for solving q2 and q3
        A = px*np.cos(q1) - d5*np.sin(theta234) + py*np.sin(q1) + d6*np.cos(theta234)*np.sin(q5)
        B = d1 - pz + d5*np.cos(theta234) + d6*np.sin(q5)*np.sin(theta234)

        # Solve q3 via cosine law (elbow angle)
        c3 = (A**2 + B**2 - a2**2 - a3**2)/(2*a2*a3)
        
        # c3 must be in the range [-1, 1] for a valid solution: (a2-a3)^2 < A^2 + B^2 < (a2+a3)^2       
        if np.abs(c3)>1:
            #c3 is outside this range, the target pose is unreachable.
            return [],-1
        
        # c3 is within [-1, 1], we can compute the elbow angle θ3.
        # if c3 is exactly 1 or -1, the robot is in a singular configuration where the elbow is fully extended or folded, respectively.
        q3 = np.arctan2(conf2*np.sqrt(1 - c3**2), c3)

        s2 = (B*a2 + B*a3*np.cos(q3) - A*a3*np.sin(q3))/(a2**2 + a3**2 + 2*a2*a3*np.cos(q3))
        c2 = (A*a2 + A*a3*np.cos(q3) + B*a3*np.sin(q3))/(a2**2 + a3**2 + 2*a2*a3*np.cos(q3))

        q2 = np.arctan2(s2, c2)
        q4 = theta234 - q2 - q3

        # Adjust for DH offsets and normalize joint angles to range [-pi, pi]
        q = np.array([q1,q2,q3,q4,q5,q6]) - self.offset
        # Limit joint angles to [-pi, pi]
        q = (q + np.pi) % (2 * np.pi) - np.pi
        status=1

        return q,status


    def joint_move(self, q_start, q_goal, steps=50):
        """
        Generate a joint-space trajectory from q_start to q_goal.

        Parameters:
        - q_start: ndarray(6,) initial joint configuration
        - q_goal: ndarray(6,) target joint configuration
        - steps: int, number of interpolation steps

        Returns:
        - q_path: ndarray(steps, 6), interpolated trajectory
        """
        return jtraj(q_start, q_goal, steps).q
        
    def cartesian_move(self, pose_start, pose_end, steps=50):
        """
        Generate a Cartesian trajectory between two poses.

        Parameters:
        - pose_start: spatialmath.SE3 initial pose
        - pose_end: spatialmath.SE3 final pose
        - steps: number of interpolation steps

        Returns:
        - list of SE3 poses interpolated using ctraj()
        """
        return ctraj(pose_start, pose_end, steps)
    
    def get_jacobian(self, q):
        """
        Returns the geometric Jacobian at joint configuration q
        """
        return self.jacob0(q)

    def nullspace(self, J):
        """
        Computes the null space of a Jacobian matrix J
        """
        return sc.linalg.null_space(J)
    
    def trajectory_along_nullspace(self, q_start, steps=100, step_size=0.01, normalize_by=0):
        """
        Generate a joint-space trajectory along the Jacobian's null space.

        Parameters:
        - q_start: ndarray(6,) initial configuration (ideally singular)
        - steps: int, number of steps
        - step_size: float, step size in joint space
        - normalize_by: int, preferred joint index for direction normalization

        Returns:
        - traj: ndarray(N, 6), the joint trajectory
        - is_internal: bool, True if singularity is internal, False if external
        """
        q_current = q_start.copy()
        trajectory = []

        for i in range(steps):
            J = self.get_jacobian(q_current)
            null = self.nullspace(J)

            if null.shape[1] == 0:
                return np.array(trajectory), False  # External singularity

            direction = null[:, 0]

            if np.abs(direction[normalize_by]) < 1e-6:
                nonzero_indices = np.where(np.abs(direction) > 1e-6)[0]
                if len(nonzero_indices) == 0:
                    return np.array(trajectory), False  # Degenerate singularity
                normalize_by = nonzero_indices[0]

            direction = -direction / direction[normalize_by]
            q_next = q_current + step_size * direction
            segment = jtraj(q_current, q_next, 2).q
            trajectory.append(segment[1])
            q_current = q_next

        return np.array(trajectory), True  # Internal singularity


    def plot_reach(self, pose):
        """ 
        Grafica el alcance del robot myCobot320 en 3D, mostrando la posición de los eslabones 
        y el alcance mínimo y máximo.
        """
        pose_aux = self._get_pose_matrix(pose)

        # Extraigo las componentes de la matriz que van a ser usadas en las ecuaciones
        px, py, pz = pose_aux.t
        nx, ny, nz = pose_aux.R[:, 0]   
        sx, sy, sz = pose_aux.R[:, 1]
        ax, ay, az = pose_aux.R[:, 2]   
        
        # Extract link lengths from DH table used in calculations
        d1 = self.links[0].d 
        a2 = self.links[1].a
        a3 = self.links[2].a
        d4 = self.links[3].d 
        d5 = self.links[4].d 
        d6 = self.links[5].d 

        # Calculo el alcance minimo y maximo
        r_min = abs(a2 - a3)
        r_max = a2 + a3

        posiciones = [np.array([0, 0, 0])]
    
        for i in range(7):
            posiciones.append(pose.A[i][:3, 3])  # Posición X, Y, Z

        pos=np.array(posiciones).T
        fig = plt.figure()
        ax3d = fig.add_subplot(111, projection='3d')

        # Posición inicial
        ax3d.plot(pos[0], pos[1], pos[2], 'o-', color='red', label='Posición')

        # Uno los puntos intermedios para visualizar los eslabones
        for i in range(len(pos[0]) - 1):
            ax3d.plot(
                [pos[0, i], pos[0, i + 1]],
                [pos[1, i], pos[1, i + 1]],
                [pos[2, i], pos[2, i + 1]],
                'r--'
            )
        muñeca = np.array([px - ax * d6,
                py - ay * d6,
                pz - az * d6])
        theta = np.linspace(0, 2 * np.pi, 100)
        x_circ = d4 * np.cos(theta)
        y_circ = d4 * np.sin(theta)
        centro_z = muñeca[2] 
        z_circ = np.full_like(theta, centro_z)

        x_max = r_max * np.cos(theta)
        y_max = r_max * np.sin(theta)

        x_min = r_min * np.cos(theta)
        y_min = r_min * np.sin(theta)


        ax3d.plot(x_circ, y_circ, z_circ,
                color='blue',
                linestyle='--',
                linewidth=2,
                label='Límite de alcance (discriminante)')
        ax3d.plot(x_max, y_max, d1, 'b--', label="Alcance Máximo")
        ax3d.plot(x_min, y_min, d1,  'r--', label="Alcance Mínimo")

        # Configuración del gráfico
        ax3d.set_xlim([-0.300, 0.300])
        ax3d.set_ylim([-0.300, 0.300])
        ax3d.set_zlim([0.000, 0.400])
        ax3d.set_xlabel('X (mm)')
        ax3d.set_ylabel('Y (mm)')
        ax3d.set_zlabel('Z (mm)')
        plt.title('myCobot320: Posición de los Eslabones')
        plt.legend()
        plt.grid(True)
        plt.show()

    def plot_conf(self, q, conf, output_path=None, mostrar=False):
        a = self.plot(q, backend='pyplot', jointaxes=False, block=False, name=False)
        fig = a.fig
        ax = fig.gca()
        ax.view_init(elev=41, azim=-37)
        ax.set_xlim([-0.05, 0.20])
        ax.set_ylim([-0.05, 0.10])
        ax.set_zlim([0.00, 0.30])
        ax.set_box_aspect([1, 1, 0.5]) 
        a.hold()
        if output_path:
            fig.savefig(output_path)
        if mostrar:
            plt.show()
        return fig

    def validate_pdcpci(self):
        q = np.random.randn(6)
        pose = self.fkine_all(q)
        conf = self.calculate_config(q).flatten()
        q_calc, _ = self.ikine_a(pose, conf, plot_reach=False)
        return q, q_calc, np.allclose(q, q_calc, atol=1e-6)
    
    def _get_pose_matrix(self, pose):
        """ 
        Extracts the final SE3 matrix from a list of joint poses (fkine_all output).
        If input is already a single SE3, returns it unchanged.
        """
        if len(pose) == 7:   
            pose_aux = pose[6]
        else:
            pose_aux = pose

        return pose_aux

__all__ = ['MyCobot320', 'SE3']