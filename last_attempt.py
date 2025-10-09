import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import keyboard
import time

STEP_SIZE = np.radians(1)

def rotate_vector(v, a, angle):
        cos_theta = np.cos(angle)
        sin_theta = np.sin(angle)
        return (v * cos_theta + np.cross(a, v) * sin_theta + a * np.dot(a, v) * (1 - cos_theta))

angle1_pos = np.radians(60)
angle2_pos = np.radians(180)
width = 45

def extract_theta_phi(v1, v2):
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

def tendons_lengths_tp(theta, phi):

    da = 2*width*np.sin(theta/2)*np.cos(phi)
    db = 2*width*np.sin(theta/2)*np.cos(phi - (2/3 * np.pi))
    dc = 2*width*np.sin(theta/2)*np.cos(phi - (4/3 * np.pi))


    return np.array([da, db, dc])

def tendons_lengths_rr(rx, ry):

    pa = np.radians(0)
    pb = np.radians(240)
    pc = np.radians(120)
    
    # da = 2*(np.sin(rx/2)*np.sin(pa) + np.sin(ry/2)*np.cos(pa))*width
    da = 2*width*(np.cos(rx/2)*np.sin(ry/2)*np.cos(pa) + np.sin(rx/2)*np.cos(ry/2)*np.sin(pa)) / np.sqrt(np.cos(rx/2)**2 * np.sin(ry/2)**2 + np.cos(ry/2)**2)
    db = 2*width*(np.cos(rx/2)*np.sin(ry/2)*np.cos(pb) + np.sin(rx/2)*np.cos(ry/2)*np.sin(pb)) / np.sqrt(np.cos(rx/2)**2 * np.sin(ry/2)**2 + np.cos(ry/2)**2)
    dc = 2*width*(np.cos(rx/2)*np.sin(ry/2)*np.cos(pc) + np.sin(rx/2)*np.cos(ry/2)*np.sin(pc)) / np.sqrt(np.cos(rx/2)**2 * np.sin(ry/2)**2 + np.cos(ry/2)**2)

    return np.array([da, db, dc])

def tp_to_rr(theta, phi):
    rx = np.arctan2(np.sin(phi) * np.tan(theta), 1)  # equivalent to arctan(tan(theta)*sin(phi))
    ry = np.arctan2(np.cos(phi) * np.tan(theta), 1)  # equivalent to arctan(tan(theta)*cos(phi))

    return rx, ry

def compute_jacobian(rx, ry, width=45, pa=np.radians(0), pb=np.radians(240), pc=np.radians(120)):
    # Helper angles
    A = np.cos(rx / 2)
    B = np.sin(rx / 2)
    C = np.cos(ry / 2)
    S = np.sin(ry / 2)

    N = np.sqrt(A**2 * S**2 + C**2)

    def derivs(p):
        cos_p = np.cos(p)
        sin_p = np.sin(p)

        num = A * S * cos_p + B * C * sin_p

        # partial derivatives of numerator
        dnum_drx = (-0.5 * np.sin(rx / 2)) * S * cos_p + (0.5 * np.cos(rx / 2)) * C * sin_p
        dnum_dry = (0.5 * np.cos(rx / 2)) * C * cos_p - (0.5 * np.sin(rx / 2)) * S * sin_p

        # partial derivatives of denominator
        dN_drx = (A * (-np.sin(rx / 2)) * S**2) / (2 * N)
        dN_dry = (A**2 * S * 0.5 * np.cos(ry / 2) - C * 0.5 * np.sin(ry / 2)) / N

        # quotient rule: d(p) = 2w * num / N
        dd_drx = 2 * width * (dnum_drx * N - num * dN_drx) / (N**2)
        dd_dry = 2 * width * (dnum_dry * N - num * dN_dry) / (N**2)

        return dd_drx, dd_dry

    # Compute for a, b, c
    dda_drx, dda_dry = derivs(pa)
    ddb_drx, ddb_dry = derivs(pb)
    ddc_drx, ddc_dry = derivs(pc)

    # Jacobian matrix (3x2)
    J = np.array([
        [dda_drx, dda_dry],
        [ddb_drx, ddb_dry],
        [ddc_drx, ddc_dry],
    ])

    return J


if __name__ == "__main__":
    angle1 = 0
    angle2 = 0

    fig1 = plt.figure()
    # fig2 = plt.figure()
    # fig3 = plt.figure()
    # fig4 = plt.figure()
    ax1 = fig1.add_subplot(111, projection='3d')
    # ax2 = fig2.add_subplot(111, projection='3d')
    # ax3 = fig3.add_subplot(111, projection='3d')
    # ax4 = fig4.add_subplot(111, projection='3d')
    

    while True:

        if keyboard.is_pressed("w"):
            angle1 += STEP_SIZE
        if keyboard.is_pressed("s"):
            angle1 -= STEP_SIZE
        if keyboard.is_pressed("d"):
            angle2 += STEP_SIZE
        if keyboard.is_pressed("a"):
            angle2 -= STEP_SIZE

        print("Angles: ", np.degrees(angle1), np.degrees(angle2))

        # -------- Theta Phi ---------

        ax1.clear()

        v1 = np.array([1, 0, 0], dtype=float)
        ax1.quiver(0, 0, 0, 0.5, 0, 0, color='red', label='x_axis')
        ax1.quiver(0, 0, 0, 0, 0.5, 0, color='blue', label='y_axis')
        ax1.quiver(0, 0, 0, 0, 0, 0.5, color='green', label='z_axis')
        r_v1 = v1.copy()
        v2 = np.array([1, 0, 0], dtype=float)
        # ax.quiver(0, 0, 0, v2[0], v2[1], v2[2], color='red', label='v2_1')
        r_v2 = v2.copy()
        z_axis = np.array([0, 0, 1], dtype=float)

        v1 = rotate_vector(v1, z_axis, angle1_pos + np.pi/2)
        ax1.quiver(0, 0, 0, v1[0], v1[1], v1[2], color='green', label='v1_proj')
        r_v1 = rotate_vector(r_v1, z_axis, angle1_pos)
        # ax.quiver(0, 0, 0, r_v1[0], r_v1[1], r_v1[2], color='yellow', label='r_v1')
        v1 = rotate_vector(v1, r_v1, angle1)
        # ax.quiver(0, 0, 0, v1[0], v1[1], v1[2], color='blue', label='v1_3')


        v2 = rotate_vector(v2, z_axis, angle2_pos + np.pi/2)
        ax1.quiver(0, 0, 0, v2[0], v2[1], v2[2], color='green', label='v2_proj')
        r_v2 = rotate_vector(r_v2, z_axis, angle2_pos)
        # ax.quiver(0, 0, 0, r_v2[0], r_v2[1], r_v2[2], color='yellow', label='r_v2')
        v2 = rotate_vector(v2, r_v2, angle2)
        # ax.quiver(0, 0, 0, v2[0], v2[1], v2[2], color='blue', label='v2_3')


        n, h = extract_theta_phi(v1, v2)
        n = n/np.linalg.norm(n)

        theta = np.arccos(np.clip(np.dot(n, z_axis), -1.0, 1.0))
        phi = np.arctan2(h[1], h[0])

        phi = (phi + np.pi/2)%(2*np.pi)

        # # v1 (blue), v2 (red), h (purple)
        ax1.quiver(0, 0, 0, v1[0], v1[1], v1[2], color='blue', label='v1')
        ax1.quiver(0, 0, 0, v2[0], v2[1], v2[2], color='blue', label='v2')
        ax1.quiver(0, 0, 0, h[0], h[1], h[2], color='purple', label='h (intersection dir)')
        ax1.quiver(0, 0, 0, n[0], n[1], n[2], color='orange', label='n (normal)')

        # Axes setup
        ax1.set_xlim([-1.5, 1.5])
        ax1.set_ylim([-1.5, 1.5])
        ax1.set_zlim([-1.5, 1.5])
        ax1.set_xlabel('X')
        ax1.set_ylabel('Y')
        ax1.set_zlabel('Z')
        ax1.set_title('Vectors v1, v2, and h')
        ax1.legend()

        print("Theta Phi: ", np.degrees(theta), np.degrees(phi))

        # # --------- Rx Ry ----------

        # ax2.clear()

        # v1 = np.array([1, 0, 0], dtype=float)
        # ax2.quiver(0, 0, 0, 0.5, 0, 0, color='red', label='x_axis')
        # ax2.quiver(0, 0, 0, 0, 0.5, 0, color='blue', label='y_axis')
        # ax2.quiver(0, 0, 0, 0, 0, 0.5, color='green', label='z_axis')
        # r_v1 = v1.copy()
        # v2 = np.array([1, 0, 0], dtype=float)
        # # ax2.quiver(0, 0, 0, v2[0], v2[1], v2[2], color='red', label='v2_1')
        # r_v2 = v2.copy()
        # z_axis = np.array([0, 0, 1], dtype=float)

        # v1 = rotate_vector(v1, z_axis, angle1_pos + np.pi/2)
        # r_v1 = rotate_vector(r_v1, z_axis, angle1_pos)
        # ax2.quiver(0, 0, 0, r_v1[0], r_v1[1], r_v1[2], color='yellow', label='r_v1')
        # v1 = rotate_vector(v1, r_v1, angle1)
        # ax2.quiver(0, 0, 0, v1[0], v1[1], v1[2], color='blue', label='v1_3')


        # v2 = rotate_vector(v2, z_axis, angle2_pos + np.pi/2)
        # r_v2 = rotate_vector(r_v2, z_axis, angle2_pos)
        # ax2.quiver(0, 0, 0, r_v2[0], r_v2[1], r_v2[2], color='yellow', label='r_v2')
        # v2 = rotate_vector(v2, r_v2, angle2)
        # ax2.quiver(0, 0, 0, v2[0], v2[1], v2[2], color='blue', label='v2_3')

        # vn = np.cross(v1, v2)
        # vn = vn/np.linalg.norm(vn)
        # ax2.quiver(0, 0, 0, vn[0], vn[1], vn[2], color='orange', label='n (normal)')

        # v_rx = np.cross(vn, np.array([1, 0, 0], dtype=float))
        # ax2.quiver(0, 0, 0, v_rx[0], v_rx[1], v_rx[2], color='purple', label='rx')

        # v_ry = np.cross(np.array([0, 1, 0], dtype=float), vn)
        # ax2.quiver(0, 0, 0, v_ry[0], v_ry[1], v_ry[2], color='brown', label='ry')

        # rx = np.arctan2(v_rx[2], v_rx[1])
        # ry = - np.arctan2(v_ry[2], v_ry[0])

        # # Axes setup
        # ax2.set_xlim([-1.5, 1.5])
        # ax2.set_ylim([-1.5, 1.5])
        # ax2.set_zlim([-1.5, 1.5])
        # ax2.set_xlabel('X')
        # ax2.set_ylabel('Y')
        # ax2.set_zlabel('Z')
        # ax2.set_title('Vectors v1, v2, and n')
        # ax2.legend()

        # print("Rx Ry: ", np.degrees(rx), np.degrees(ry))

        rx, ry = tp_to_rr(theta, phi)
        print("Converted Rx Ry: ", np.degrees(rx), np.degrees(ry))

        # ---------- Compare --------
        # ax3.clear()
        
        # ax3.quiver(0, 0, 0, 0.5, 0, 0, color='red', label='x_axis')
        # ax3.quiver(0, 0, 0, 0, 0.5, 0, color='blue', label='y_axis')
        # ax3.quiver(0, 0, 0, 0, 0, 0.5, color='green', label='z_axis')

        # ax3.quiver(0, 0, 0, n[0], n[1], n[2], color='orange', label='n tp (normal)')
        # ax3.quiver(0, 0, 0, vn[0], vn[1], vn[2], color='green', label='n rr (normal)')

        # # Axes setup
        # ax3.set_ylim([-1.5, 1.5])
        # ax3.set_xlim([-1.5, 1.5])
        # ax3.set_zlim([-1.5, 1.5])
        # ax3.set_xlabel('X')
        # ax3.set_ylabel('Y')
        # ax3.set_zlabel('Z')
        # ax3.set_title('Vectors v1, v2, and n')
        # ax3.legend()

        l1 = tendons_lengths_tp(theta, phi)
        l2 = tendons_lengths_rr(rx, ry)
        print("l1: ", np.round(l1, 2))
        print("l2: ", np.round(l2, 2))

        J = compute_jacobian(rx, ry)
        print("J: \n", np.round(J, 4))


        plt.tight_layout()
        plt.draw()
        plt.pause(0.0001) 

        time.sleep(0.1)
