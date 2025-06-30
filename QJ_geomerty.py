import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class QJgeomerty:
    def __init__(self, width):
        self.theta = 0
        self.phi = 0

        self.da = 0
        self.db = 0
        self.dc = 0

        self.width = width

        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')

    def read_angle(self):
        return self.theta, self.phi
    
    def set_angle(self, theta, phi):
        self.theta = theta
        self.phi = phi

        self.da = 2*self.width*np.sin(self.theta/2)*np.cos(self.phi)
        self.db = 2*self.width*np.sin(self.theta/2)*np.cos(self.phi + (2/3 * np.pi))
        self.dc = 2*self.width*np.sin(self.theta/2)*np.cos(self.phi+ (4/3 * np.pi))


        return self.da, self.db, self.dc
    
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

    def compute_theta_phi(self, angle1, angle1_pos, angle2, angle2_pos):

        self.ax.clear()

        v1 = np.array([1, 0, 0], dtype=float)
        self.ax.quiver(0, 0, 0, 2, 0, 0, color='red', label='x_axis')
        # self.ax.quiver(0, 0, 0, 0, 2, 0, color='red', label='y_axis')
        # self.ax.quiver(0, 0, 0, 0, 0, 2, color='red', label='z_axis')
        r_v1 = v1.copy()
        v2 = np.array([1, 0, 0], dtype=float)
        # ax.quiver(0, 0, 0, v2[0], v2[1], v2[2], color='red', label='v2_1')
        r_v2 = v2.copy()
        z_axis = np.array([0, 0, 1], dtype=float)

        v1 = self.rotate_vector(v1, z_axis, angle1_pos)
        self.ax.quiver(0, 0, 0, v1[0], v1[1], v1[2], color='green', label='v1_proj')
        r_v1 = self.rotate_vector(r_v1, z_axis, angle1_pos - np.pi / 2)
        # ax.quiver(0, 0, 0, r_v1[0], r_v1[1], r_v1[2], color='yellow', label='r_v1')
        v1 = self.rotate_vector(v1, r_v1, angle1)
        # ax.quiver(0, 0, 0, v1[0], v1[1], v1[2], color='blue', label='v1_3')


        v2 = self.rotate_vector(v2, z_axis, angle2_pos)
        self.ax.quiver(0, 0, 0, v2[0], v2[1], v2[2], color='green', label='v2_proj')
        r_v2 = self.rotate_vector(r_v2, z_axis, angle2_pos - np.pi / 2)
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
        