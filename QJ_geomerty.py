import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import keyboard
import time

STEP_SIZE = 0.01
DELAY = 0.05

class QJgeomerty:
    def __init__(self, width, angle1_pos=np.deg2rad(60), angle2_pos=np.deg2rad(180)):
        self.theta = 0
        self.phi = 0

        self.da = 0
        self.db = 0
        self.dc = 0
        self.damping = 0.005

        self.width = width

        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')

        z_axis = np.array([0, 0, 1], dtype=float)
        self.encoder1_axis = self.rotate_vector(np.array([1, 0, 0], dtype=float), z_axis, angle1_pos)
        self.encoder2_axis = self.rotate_vector(np.array([1, 0, 0], dtype=float), z_axis, angle2_pos)
        self.encoder1_v = self.rotate_vector(self.encoder1_axis, z_axis, np.pi/2)
        self.encoder2_v = self.rotate_vector(self.encoder2_axis, z_axis, np.pi/2)

    def read_angle(self):
        return self.theta, self.phi
    
    def tendons_lengths_tp(self, theta, phi):
        self.theta = theta
        self.phi = phi

        self.da = 2*self.width*np.sin(self.theta/2)*np.cos(self.phi)
        self.db = 2*self.width*np.sin(self.theta/2)*np.cos(self.phi - (2/3 * np.pi))
        self.dc = 2*self.width*np.sin(self.theta/2)*np.cos(self.phi - (4/3 * np.pi))


        return self.da, self.db, self.dc
    
    def tendons_lengths_rr(self, rx, ry):

        pa = np.radians(0)
        pb = np.radians(-120)
        pc = np.radians(-240)
        
        # da = 2*(np.sin(rx/2)*np.sin(pa) + np.sin(ry/2)*np.cos(pa))*self.width
        da = 2*self.width*(np.cos(rx/2)*np.sin(ry/2)*np.cos(pa) + np.sin(rx/2)*np.cos(ry/2)*np.sin(pa)) / np.sqrt(np.cos(rx/2)**2 * np.sin(ry/2)**2 + np.cos(ry/2)**2)
        db = 2*self.width*(np.cos(rx/2)*np.sin(ry/2)*np.cos(pb) + np.sin(rx/2)*np.cos(ry/2)*np.sin(pb)) / np.sqrt(np.cos(rx/2)**2 * np.sin(ry/2)**2 + np.cos(ry/2)**2)
        dc = 2*self.width*(np.cos(rx/2)*np.sin(ry/2)*np.cos(pc) + np.sin(rx/2)*np.cos(ry/2)*np.sin(pc)) / np.sqrt(np.cos(rx/2)**2 * np.sin(ry/2)**2 + np.cos(ry/2)**2)

        return da, db, dc
    

    def partial(self, rx, ry, pa):
        # Precompute sines and cosines
        cos_rx2 = np.cos(rx / 2)
        sin_rx2 = np.sin(rx / 2)
        cos_ry2 = np.cos(ry / 2)
        sin_ry2 = np.sin(ry / 2)
        w=self.width

        # Numerator and denominator
        N = cos_rx2 * sin_ry2 * np.cos(pa) + sin_rx2 * cos_ry2 * np.sin(pa)
        D = np.sqrt(cos_rx2**2 * sin_ry2**2 + cos_ry2**2)

        # Derivatives of numerator
        dN_drx = -0.5 * sin_rx2 * sin_ry2 * np.cos(pa) + 0.5 * cos_rx2 * cos_ry2 * np.sin(pa)
        dN_dry = 0.5 * cos_rx2 * cos_ry2 * np.cos(pa) - 0.5 * sin_rx2 * sin_ry2 * np.sin(pa)

        # Derivatives of denominator
        dD_drx = -0.5 * sin_rx2 * cos_rx2 * sin_ry2**2 / D
        dD_dry = 0.5 * sin_ry2 * cos_ry2 * (cos_rx2**2 - 1) / D  # simplifies to -0.5 * sin_ry2 * cos_ry2 * sin_rx2**2 / D

        # Partial derivatives of a
        da_drx = 2 * w * (dN_drx * D - N * dD_drx) / D**2
        da_dry = 2 * w * (dN_dry * D - N * dD_dry) / D**2

        return da_drx, da_dry
    

    def jacobian_RR(self, rx, ry):

        pa = np.radians(0)
        pb = np.radians(-120)
        pc = np.radians(-240)

        da_dx, da_dy = self.partial(rx, ry, pa)
        db_dx, db_dy = self.partial(rx, ry, pb)
        dc_dx, dc_dy = self.partial(rx, ry, pc)

        # Jacobian matrix
        J = np.array([
            [da_dx, da_dy],
            [db_dx, db_dy],
            [dc_dx, dc_dy]
        ])
        return J

    def jacobian_tp(self, theta, phi):

        q = np.array([theta, phi])
        
        da_dtheta = self.width * np.cos(q[0]/2) * np.cos(q[1])
        da_dphi   = -2 * self.width * np.sin(q[0]/2) * np.sin(q[1])
        db_dtheta = self.width * np.cos(q[0]/2) * np.cos(q[1] - 2*np.pi/3)
        db_dphi   = -2 * self.width * np.sin(q[0]/2) * np.sin(q[1] - 2*np.pi/3)
        dc_dtheta = self.width * np.cos(q[0]/2) * np.cos(q[1] - 4*np.pi/3)
        dc_dphi   = -2 * self.width * np.sin(q[0]/2) * np.sin(q[1] - 4*np.pi/3)

        return np.array([[da_dtheta, da_dphi], [db_dtheta, db_dphi], [dc_dtheta, dc_dphi]])

    
    def read_tendons_d(self):
        return self.da, self.db, self.dc
    
    def rotate_vector(self, v, a, angle):
        v = np.array(v)
        a = np.array(a)
        a = a / np.linalg.norm(a)
        cos_theta = np.cos(angle)
        sin_theta = np.sin(angle)
        return (v * cos_theta +
                np.cross(a, v) * sin_theta +
                a * np.dot(a, v) * (1 - cos_theta))
    
    def rotate_vector_fast(self, v, a, angle):
        cos_theta = np.cos(angle)
        sin_theta = np.sin(angle)
        return (v * cos_theta + np.cross(a, v) * sin_theta + a * np.dot(a, v) * (1 - cos_theta))

    def extract_theta_phi(self, v1, v2):
        """
        Given two 3D vectors, computes:
        - The vector `h`: direction of the intersection line between the plane formed by v1 & v2 and the XY plane.
        - The angle between the plane's normal and the Z-axis.
        """
        v1 = np.asarray(v1, dtype=float)
        v2 = np.asarray(v2, dtype=float)

        # Normal vector of the plane
        n = np.cross(v1, v2)
        norm_n = np.linalg.norm(n)
        if norm_n == 0:
            raise ValueError("Vectors are colinear; no unique plane defined.")
        n_unit = n / norm_n

        # Z-axis
        z_axis = np.array([0.0, 0.0, 1.0])

        # Intersection line between the plane and XY plane
        h = np.cross(n_unit, z_axis)
        h_norm = np.linalg.norm(h)
        if h_norm < 1e-8:
            h = np.array([1.0, 0.0, 0.0])  # Default direction if plane is parallel to XY

        else:
            h = h / h_norm

        return n_unit, h
    
    def extract_theta_phi_fast(self, v1, v2):
        """
        Given two 3D vectors, computes:
        - The vector `h`: direction of the intersection line between the plane formed by v1 & v2 and the XY plane.
        - The angle between the plane's normal and the Z-axis.
        """

        # Normal vector
        n = np.cross(v1, v2)
        n_unit = n/np.linalg.norm(n)

        # Intersection line
        h = np.cross(n, np.array([0.0, 0.0, 1.0]))

        # Singularity
        if np.linalg.norm(h) < 1e-8:
            h = np.array([1.0, 0.0, 0.0])  # Default direction if plane is parallel to XY

        return n_unit, h
    
    def compute_theta_phi(self, angle1, angle2):


        v1 = self.rotate_vector_fast(self.encoder1_v, self.encoder1_axis, angle1)
        v2 = self.rotate_vector_fast(self.encoder2_v, self.encoder2_axis, angle2)

        n, h = self.extract_theta_phi_fast(v1, v2)

        theta = np.arccos(np.clip(np.dot(n, np.array([0, 0, 1])), -1.0, 1.0))
        phi = np.arctan2(h[1], h[0])

        phi = (phi + np.pi/2) % (2 * np.pi)

        return theta, phi

    def compute_theta_phi_dysplay(self, angle1, angle1_pos, angle2, angle2_pos):

        self.ax.clear()

        v1 = np.array([1, 0, 0], dtype=float)
        self.ax.quiver(0, 0, 0, 0.5, 0, 0, color='red', label='x_axis')
        self.ax.quiver(0, 0, 0, 0, 0.5, 0, color='blue', label='y_axis')
        self.ax.quiver(0, 0, 0, 0, 0, 0.5, color='green', label='z_axis')
        r_v1 = v1.copy()
        v2 = np.array([1, 0, 0], dtype=float)
        # ax.quiver(0, 0, 0, v2[0], v2[1], v2[2], color='red', label='v2_1')
        r_v2 = v2.copy()
        z_axis = np.array([0, 0, 1], dtype=float)

        v1 = self.rotate_vector(v1, z_axis, angle1_pos + np.pi/2)
        self.ax.quiver(0, 0, 0, v1[0], v1[1], v1[2], color='green', label='v1_proj')
        r_v1 = self.rotate_vector(r_v1, z_axis, angle1_pos)
        # ax.quiver(0, 0, 0, r_v1[0], r_v1[1], r_v1[2], color='yellow', label='r_v1')
        v1 = self.rotate_vector(v1, r_v1, angle1)
        # ax.quiver(0, 0, 0, v1[0], v1[1], v1[2], color='blue', label='v1_3')


        v2 = self.rotate_vector(v2, z_axis, angle2_pos + np.pi/2)
        self.ax.quiver(0, 0, 0, v2[0], v2[1], v2[2], color='green', label='v2_proj')
        r_v2 = self.rotate_vector(r_v2, z_axis, angle2_pos)
        # ax.quiver(0, 0, 0, r_v2[0], r_v2[1], r_v2[2], color='yellow', label='r_v2')
        v2 = self.rotate_vector(v2, r_v2, angle2)
        # ax.quiver(0, 0, 0, v2[0], v2[1], v2[2], color='blue', label='v2_3')


        n, h = self.extract_theta_phi(v1, v2)

        theta = np.arccos(np.clip(np.dot(n, z_axis), -1.0, 1.0))
        phi = np.arctan2(h[1], h[0])

        # # v1 (blue), v2 (red), h (purple)
        self.ax.quiver(0, 0, 0, v1[0], v1[1], v1[2], color='blue', label='v1')
        self.ax.quiver(0, 0, 0, v2[0], v2[1], v2[2], color='blue', label='v2')
        self.ax.quiver(0, 0, 0, h[0], h[1], h[2], color='purple', label='h (intersection dir)')
        self.ax.quiver(0, 0, 0, n[0], n[1], n[2], color='orange', label='n (normal)')

        # Axes setup
        self.ax.set_xlim([-1.5, 1.5])
        self.ax.set_ylim([-1.5, 1.5])
        self.ax.set_zlim([-1.5, 1.5])
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        self.ax.set_title('Vectors v1, v2, and h')
        self.ax.legend()
        plt.tight_layout()
        plt.draw()
        plt.pause(0.0001)

        return theta, phi
    
    def compute_rx_ry_dysplay(self, angle1, angle1_pos, angle2, angle2_pos):

        self.ax.clear()

        v1 = np.array([1, 0, 0], dtype=float)
        self.ax.quiver(0, 0, 0, 0.5, 0, 0, color='red', label='x_axis')
        self.ax.quiver(0, 0, 0, 0, 0.5, 0, color='blue', label='y_axis')
        self.ax.quiver(0, 0, 0, 0, 0, 0.5, color='green', label='z_axis')
        r_v1 = v1.copy()
        v2 = np.array([1, 0, 0], dtype=float)
        # self.ax.quiver(0, 0, 0, v2[0], v2[1], v2[2], color='red', label='v2_1')
        r_v2 = v2.copy()
        z_axis = np.array([0, 0, 1], dtype=float)

        v1 = self.rotate_vector(v1, z_axis, angle1_pos + np.pi/2)
        r_v1 = self.rotate_vector(r_v1, z_axis, angle1_pos)
        self.ax.quiver(0, 0, 0, r_v1[0], r_v1[1], r_v1[2], color='yellow', label='r_v1')
        v1 = self.rotate_vector(v1, r_v1, angle1)
        self.ax.quiver(0, 0, 0, v1[0], v1[1], v1[2], color='blue', label='v1_3')


        v2 = self.rotate_vector(v2, z_axis, angle2_pos + np.pi/2)
        r_v2 = self.rotate_vector(r_v2, z_axis, angle2_pos)
        self.ax.quiver(0, 0, 0, r_v2[0], r_v2[1], r_v2[2], color='yellow', label='r_v2')
        v2 = self.rotate_vector(v2, r_v2, angle2)
        self.ax.quiver(0, 0, 0, v2[0], v2[1], v2[2], color='blue', label='v2_3')

        n = np.cross(v1, v2)
        self.ax.quiver(0, 0, 0, n[0], n[1], n[2], color='orange', label='n (normal)')

        v_rx = np.cross(n, np.array([1, 0, 0], dtype=float))
        self.ax.quiver(0, 0, 0, v_rx[0], v_rx[1], v_rx[2], color='purple', label='rx')

        v_ry = np.cross(np.array([0, 1, 0], dtype=float), n)
        self.ax.quiver(0, 0, 0, v_ry[0], v_ry[1], v_ry[2], color='brown', label='ry')

        rx = np.arctan2(v_rx[2], v_rx[1])
        ry = - np.arctan2(v_ry[2], v_ry[0])

        # Axes setup
        self.ax.set_xlim([-1.5, 1.5])
        self.ax.set_ylim([-1.5, 1.5])
        self.ax.set_zlim([-1.5, 1.5])
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        self.ax.set_title('Vectors v1, v2, and n')
        self.ax.legend()
        plt.tight_layout()
        plt.draw()
        plt.pause(0.0001)

        return rx, ry
    
    def compute_rx_ry(self, angle1, angle2):

        
        v1 = self.rotate_vector_fast(self.encoder1_v, self.encoder1_axis, angle1)
        v2 = self.rotate_vector_fast(self.encoder2_v, self.encoder2_axis, angle2)

        n = np.cross(v1, v2)
        
        v_rx = np.cross(n, np.array([1, 0, 0], dtype=float))
        v_ry = np.cross(np.array([0, 1, 0], dtype=float), n)
        
        rx = np.arctan2(v_rx[2], v_rx[1])
        ry = - np.arctan2(v_ry[2], v_ry[0])

        # self.ax.clear()
        # self.ax.quiver(0, 0, 0, 0.5, 0, 0, color='red', label='x_axis')
        # self.ax.quiver(0, 0, 0, 0, 0.5, 0, color='blue', label='y_axis')
        # self.ax.quiver(0, 0, 0, 0, 0, 0.5, color='green', label='z_axis')
        # self.ax.quiver(0, 0, 0, v1[0], v1[1], v1[2], color='blue', label='v1')
        # self.ax.quiver(0, 0, 0, v2[0], v2[1], v2[2], color='blue', label='v2')

        # self.ax.quiver(0, 0, 0, n[0], n[1], n[2], color='orange', label='n (normal)')

        # self.ax.quiver(0, 0, 0, v_rx[0], v_rx[1], v_rx[2], color='purple', label='rx')

        # self.ax.quiver(0, 0, 0, v_ry[0], v_ry[1], v_ry[2], color='brown', label='ry')

        # # Axes setup
        # self.ax.set_xlim([-1.5, 1.5])
        # self.ax.set_ylim([-1.5, 1.5])
        # self.ax.set_zlim([-1.5, 1.5])
        # self.ax.set_xlabel('X')
        # self.ax.set_ylabel('Y')
        # self.ax.set_zlabel('Z')
        # self.ax.set_title('Vectors v1, v2, and n')
        # self.ax.legend()
        # plt.tight_layout()
        # plt.draw()
        # plt.pause(0.0001)

        return rx, ry
    
    
    def angle_wrap(self, e):
        return (e + np.pi) % (2 * np.pi) - np.pi
    
    def damped_least_square_inverse(self, J, tau_des):
        JT = J.T
        tau_vec = tau_des.reshape(2,1)
        f_pinv = np.linalg.inv(J @ J.T + self.damping * np.eye(J.shape[0])) @ J @ tau_vec

        return f_pinv.flatten()

if __name__ == "__main__":
    qj = QJgeomerty(width=50)

    angle1 = 0
    angle2 = 0

    q_des = np.array([0, 0])

    while True:

        if keyboard.is_pressed("w"):
            angle1 += STEP_SIZE
        if keyboard.is_pressed("s"):
            angle1 -= STEP_SIZE
        if keyboard.is_pressed("d"):
            angle2 += STEP_SIZE
        if keyboard.is_pressed("a"):
            angle2 -= STEP_SIZE

        # print("angle1 : ", np.rad2deg(angle1), " angle2 : ", np.rad2deg(angle2))

        # theta, phi = qj.compute_theta_phi_dysplay(angle1=2*angle1, angle1_pos=np.deg2rad(60), angle2=2*angle2, angle2_pos=np.deg2rad(180))

        # print("theta : ", np.rad2deg(theta), " phi : ", np.rad2deg(phi))

        rx, ry = qj.compute_rx_ry_dysplay(angle1=2*angle1, angle1_pos=np.deg2rad(60), angle2=2*angle2, angle2_pos=np.deg2rad(180))
        q_rr = np.array([rx, ry])

        print("rx : ", np.rad2deg(rx), " ry : ", np.rad2deg(ry))

        # t1 = time.time()
        theta, phi = qj.compute_theta_phi(angle1=2*angle1, angle2=2*angle2)
        q_tp = np.array([theta, phi])
  
        # t2 = time.time()
        # # rx, ry = qj.compute_rx_ry(angle1=2*angle1, angle2=2*angle2)
        # t3 = time.time()

        # # print("Fast theta : ", np.rad2deg(theta), " Fast phi : ", np.rad2deg(phi))
        # # print("Fast rx : ", np.rad2deg(rx), " Fast ry : ", np.rad2deg(ry))

        # # print("Execution time us: ", (t2-t1)*1000000, ", ", (t3-t2)*1000000)

        # datp, dbtp, dctp = qj.tendons_lengths_tp(theta, phi)
        # darr, dbrr, dcrr = qj.tendons_lengths_rr(rx, ry)
        JRR = qj.jacobian_RR(rx, ry)
        JTP = qj.jacobian_tp(theta, phi)

        # print("Tendon lengths theta phi: ", datp, " ", dbtp, " ", dctp)
        # print("Tendon lengths rx ry: ", darr, " ", dbrr, " ", dcrr)
        # print("JRR: ", JRR)
        # print("JTP: ", JTP)

        e_rr = qj.angle_wrap(q_des-q_rr)
        e_tp = qj.angle_wrap(q_des-q_tp)


        f_rr = qj.damped_least_square_inverse(JRR, e_rr)
        f_tp = qj.damped_least_square_inverse(JTP, e_tp)

        print("f_rr: ", f_rr)
        print("f_tp: ", f_tp)

        time.sleep(0.1)


        

        
