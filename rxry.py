import numpy as np
import sympy as sp

width = 10
rn = 2
z_axis = np.array([0, 0, 1], dtype=float)

angle1_pos=np.deg2rad(60)
angle2_pos=np.deg2rad(180)

def rotate_vector_fast(v, a, angle):
        cos_theta = np.cos(angle)
        sin_theta = np.sin(angle)
        return (v * cos_theta + np.cross(a, v) * sin_theta + a * np.dot(a, v) * (1 - cos_theta))

encoder1_axis = rotate_vector_fast(np.array([1, 0, 0], dtype=float), z_axis, angle1_pos)
encoder2_axis = rotate_vector_fast(np.array([1, 0, 0], dtype=float), z_axis, angle2_pos)
encoder1_v = rotate_vector_fast(encoder1_axis, z_axis, np.pi/2)
encoder2_v = rotate_vector_fast(encoder2_axis, z_axis, np.pi/2)

def tendons_lengths_tp(theta, phi):
    theta = theta
    phi = phi

    da = 2*width*np.sin(theta/2)*np.cos(phi)
    db = 2*width*np.sin(theta/2)*np.cos(phi - (2/3 * np.pi))
    dc = 2*width*np.sin(theta/2)*np.cos(phi - (4/3 * np.pi))


    return np.array([da, db, dc])

def tendons_lengths_rr(rx, ry):

    pa = np.radians(0)
    pb = np.radians(120)
    pc = np.radians(240)
    
    # da = 2*(np.sin(rx/2)*np.sin(pa) + np.sin(ry/2)*np.cos(pa))*width
    da = 2*width*(np.cos(rx/2)*np.sin(ry/2)*np.cos(pa) + np.sin(rx/2)*np.cos(ry/2)*np.sin(pa)) / np.sqrt(np.cos(rx/2)**2 * np.sin(ry/2)**2 + np.cos(ry/2)**2)
    db = 2*width*(np.cos(rx/2)*np.sin(ry/2)*np.cos(pb) + np.sin(rx/2)*np.cos(ry/2)*np.sin(pb)) / np.sqrt(np.cos(rx/2)**2 * np.sin(ry/2)**2 + np.cos(ry/2)**2)
    dc = 2*width*(np.cos(rx/2)*np.sin(ry/2)*np.cos(pc) + np.sin(rx/2)*np.cos(ry/2)*np.sin(pc)) / np.sqrt(np.cos(rx/2)**2 * np.sin(ry/2)**2 + np.cos(ry/2)**2)

    return np.array([da, db, dc])
    

def tendon_lengths_test(rx, ry):
    # rx, ry are in radians
    A = np.sqrt(1 + np.tan(rx)**2 + np.tan(ry)**2)
    B = np.sqrt(np.tan(rx)**2 + np.tan(ry)**2)

    # 1) sin(theta/2) * cos(phi)
    val1 = 2 * width * np.sqrt((A - 1) / (2 * A)) * (np.tan(ry) / B)

    # 2) sin(theta/2) * cos(phi - 2/3 * pi)
    val2 = 2 * width * np.sqrt((A - 1) / (2 * A)) * ((-np.tan(ry) + np.sqrt(3) * np.tan(rx)) / (2 * B))

    # 3) sin(theta/2) * cos(phi - 4/3 * pi)
    val3 = 2 * width * np.sqrt((A - 1) / (2 * A)) * ((-np.tan(ry) - np.sqrt(3) * np.tan(rx)) / (2 * B))

    return np.array([val1, val2, val3])

def lengths_tendons_sympy(rx, ry):
    
    distances = np.array([], dtype=float)

    rx = rx/2
    ry = ry/2

    for a in [0, -(2/3 * np.pi), (2/3 * np.pi)]:
        d = ((width*np.sin(a)*np.sin(ry) + np.sin(rx)*np.cos(a))*np.cos(ry))/np.sqrt(np.sin(rx)**2 - np.sin(ry)**4 + np.sin(ry)**2)
        d = d
        distances = np.append(distances, d)

    return distances

def tp_to_rr(theta, phi):
    rx = np.arctan2(np.sin(phi) * np.tan(theta), 1)  # equivalent to arctan(tan(theta)*sin(phi))
    ry = np.arctan2(np.cos(phi) * np.tan(theta), 1)  # equivalent to arctan(tan(theta)*cos(phi))

    return rx, ry

def tendons_jacobian(theta, phi):
    q = np.array([theta, phi])
    
    da_dtheta = width * np.cos(q[0]/2) * np.cos(q[1])
    da_dphi   = -2 * width * np.sin(q[0]/2) * np.sin(q[1])
    db_dtheta = width * np.cos(q[0]/2) * np.cos(q[1] - 2*np.pi/3)
    db_dphi   = -2 * width * np.sin(q[0]/2) * np.sin(q[1] - 2*np.pi/3)
    dc_dtheta = width * np.cos(q[0]/2) * np.cos(q[1] - 4*np.pi/3)
    dc_dphi   = -2 * width * np.sin(q[0]/2) * np.sin(q[1] - 4*np.pi/3)

    return np.array([[da_dtheta, da_dphi], [db_dtheta, db_dphi], [dc_dtheta, dc_dphi]])

def _tendons_jacobian_rr(q):

    x = q[0]
    y = q[1]

    w = width

    # d/rx val1
    dx_1 = -(w * np.tan(y) * 1/np.cos(x)**2 * np.tan(x) * (np.tan(x)**2 * (2 * np.sqrt(np.tan(x)**2 + np.tan(y)**2 + 1) - 3) + np.tan(y)**2 * (2 * np.sqrt(np.tan(x)**2 + np.tan(y)**2 + 1) - 3) + 2 * np.sqrt(np.tan(x)**2 + np.tan(y)**2 + 1) - 2)) / (np.sqrt(2) * (np.tan(x)**2 + np.tan(y)**2)**(3 / 2) * (np.tan(x)**2 + np.tan(y)**2 + 1)**(5 / 4) * np.sqrt(np.sqrt(np.tan(x)**2 + np.tan(y)**2 + 1) - 1))
    # d/ry val1
    dy_1 = -(w * 1/np.cos(x)**2 * np.tan(x) * 1/np.cos(y)**2 * (8 * np.tan(y)**2 * (np.tan(y)**2 + np.tan(x)**2 + 1)**(5 / 2) - 4 * np.tan(x)**2 * (np.tan(y)**2 + np.tan(x)**2 + 1)**(5 / 2) - 2 * np.tan(y)**4 * (np.tan(y)**2 + np.tan(x)**2 + 1)**(3 / 2) - 2 * np.tan(x)**2 * np.tan(y)**2 * (np.tan(y)**2 + np.tan(x)**2 + 1)**(3 / 2) + 18 * np.tan(y)**6 * np.sqrt(np.tan(y)**2 + np.tan(x)**2 + 1) + 30 * np.tan(x)**2 * np.tan(y)**4 * np.sqrt(np.tan(y)**2 + np.tan(x)**2 + 1) + 20 * np.tan(y)**4 * np.sqrt(np.tan(y)**2 + np.tan(x)**2 + 1) + 6 * np.tan(x)**4 * np.tan(y)**2 * np.sqrt(np.tan(y)**2 + np.tan(x)**2 + 1) + 10 * np.tan(x)**2 * np.tan(y)**2 * np.sqrt(np.tan(y)**2 + np.tan(x)**2 + 1) + 8 * np.tan(y)**2 * np.sqrt(np.tan(y)**2 + np.tan(x)**2 + 1) - 6 * np.tan(x)**6 * np.sqrt(np.tan(y)**2 + np.tan(x)**2 + 1) - 10 * np.tan(x)**4 * np.sqrt(np.tan(y)**2 + np.tan(x)**2 + 1) - 4 * np.tan(x)**2 * np.sqrt(np.tan(y)**2 + np.tan(x)**2 + 1) - 8 * np.tan(y)**8 - 20 * np.tan(x)**2 * np.tan(y)**6 - 39 * np.tan(y)**6 - 12 * np.tan(x)**4 * np.tan(y)**4 - 60 * np.tan(x)**2 * np.tan(y)**4 - 42 * np.tan(y)**4 + 4 * np.tan(x)**6 * np.tan(y)**2 - 3 * np.tan(x)**4 * np.tan(y)**2 - 20 * np.tan(x)**2 * np.tan(y)**2 - 16 * np.tan(y)**2 + 2 * np.tan(x)**2 * (np.tan(x)**2 + 1) * (2 * np.tan(x)**4 + 7 * np.tan(x)**2 + 4))) / (2**(3 / 2) * (np.tan(y)**2 + np.tan(x)**2)**(5 / 2) * (np.tan(y)**2 + np.tan(x)**2 + 1)**(9 / 4) * (np.sqrt(np.tan(y)**2 + np.tan(x)**2 + 1) - 1)**(3 / 2))

    # d/rx val2
    dx_2 = (w * 1/np.cos(x)**2 * (np.tan(x) * (np.tan(y) * (2 * np.sqrt(np.tan(x)**2 + np.tan(y)**2 + 1) - 2) + np.tan(y)**3 * (2 * np.sqrt(np.tan(x)**2 + np.tan(y)**2 + 1) - 3)) + np.sqrt(3) * np.tan(y)**2 * np.tan(x)**2 * (2 * np.sqrt(np.tan(x)**2 + np.tan(y)**2 + 1) - 1) + np.sqrt(3) * np.tan(y)**4 * (2 * np.sqrt(np.tan(x)**2 + np.tan(y)**2 + 1) - 2) + np.sqrt(3) * np.tan(y)**2 * (2 * np.sqrt(np.tan(x)**2 + np.tan(y)**2 + 1) - 2) + np.tan(y) * np.tan(x)**3 * (2 * np.sqrt(np.tan(x)**2 + np.tan(y)**2 + 1) - 3) + np.sqrt(3) * np.tan(x)**4)) / (2**(3 / 2) * (np.tan(x)**2 + np.tan(y)**2)**(3 / 2) * (np.tan(x)**2 + np.tan(y)**2 + 1)**(5 / 4) * np.sqrt(np.sqrt(np.tan(x)**2 + np.tan(y)**2 + 1) - 1))
    # d/rx val2
    dy_2 = -(3 * w * 1/np.cos(x)**2 * 1/np.cos(y)**2 * np.tan(y) * (np.sqrt(np.tan(y)**2 + np.tan(x)**2 + 1) * (2 * np.sqrt(3) * np.tan(y)**4 + 2 * np.tan(x) * np.tan(y)**3 + (2 * np.sqrt(3) * np.tan(x)**2 + 2 * np.sqrt(3)) * np.tan(y)**2 + (2 * np.tan(x)**3 + 2 * np.tan(x)) * np.tan(y)) - 2 * np.sqrt(3) * np.tan(y)**4 - 3 * np.tan(x) * np.tan(y)**3 + (-np.sqrt(3) * np.tan(x)**2 - 2 * np.sqrt(3)) * np.tan(y)**2 + (-3 * np.tan(x)**3 - 2 * np.tan(x)) * np.tan(y) + np.sqrt(3) * np.tan(x)**4)) / (2**(3 / 2) * (np.tan(y)**2 + np.tan(x)**2)**(5 / 2) * (np.tan(y)**2 + np.tan(x)**2 + 1)**(5 / 4) * np.sqrt(np.sqrt(np.tan(y)**2 + np.tan(x)**2 + 1) - 1)) - (5 * w * 1/np.cos(x)**2 * 1/np.cos(y)**2 * np.tan(y) * (np.sqrt(np.tan(y)**2 + np.tan(x)**2 + 1) * (2 * np.sqrt(3) * np.tan(y)**4 + 2 * np.tan(x) * np.tan(y)**3 + (2 * np.sqrt(3) * np.tan(x)**2 + 2 * np.sqrt(3)) * np.tan(y)**2 + (2 * np.tan(x)**3 + 2 * np.tan(x)) * np.tan(y)) - 2 * np.sqrt(3) * np.tan(y)**4 - 3 * np.tan(x) * np.tan(y)**3 + (-np.sqrt(3) * np.tan(x)**2 - 2 * np.sqrt(3)) * np.tan(y)**2 + (-3 * np.tan(x)**3 - 2 * np.tan(x)) * np.tan(y) + np.sqrt(3) * np.tan(x)**4)) / (2**(5 / 2) * (np.tan(y)**2 + np.tan(x)**2)**(3 / 2) * (np.tan(y)**2 + np.tan(x)**2 + 1)**(9 / 4) * np.sqrt(np.sqrt(np.tan(y)**2 + np.tan(x)**2 + 1) - 1)) + (w * 1/np.cos(x)**2 * ((1/np.cos(y)**2 * np.tan(y) * (2 * np.sqrt(3) * np.tan(y)**4 + 2 * np.tan(x) * np.tan(y)**3 + (2 * np.sqrt(3) * np.tan(x)**2 + 2 * np.sqrt(3)) * np.tan(y)**2 + (2 * np.tan(x)**3 + 2 * np.tan(x)) * np.tan(y))) / np.sqrt(np.tan(y)**2 + np.tan(x)**2 + 1) + np.sqrt(np.tan(y)**2 + np.tan(x)**2 + 1) * (8 * np.sqrt(3) * 1/np.cos(y)**2 * np.tan(y)**3 + 6 * np.tan(x) * 1/np.cos(y)**2 * np.tan(y)**2 + 2 * (2 * np.sqrt(3) * np.tan(x)**2 + 2 * np.sqrt(3)) * 1/np.cos(y)**2 * np.tan(y) + (2 * np.tan(x)**3 + 2 * np.tan(x)) * 1/np.cos(y)**2) - 8 * np.sqrt(3) * 1/np.cos(y)**2 * np.tan(y)**3 - 9 * np.tan(x) * 1/np.cos(y)**2 * np.tan(y)**2 + 2 * (-np.sqrt(3) * np.tan(x)**2 - 2 * np.sqrt(3)) * 1/np.cos(y)**2 * np.tan(y) + (-3 * np.tan(x)**3 - 2 * np.tan(x)) * 1/np.cos(y)**2)) / (2**(3 / 2) * (np.tan(y)**2 + np.tan(x)**2)**(3 / 2) * (np.tan(y)**2 + np.tan(x)**2 + 1)**(5 / 4) * np.sqrt(np.sqrt(np.tan(y)**2 + np.tan(x)**2 + 1) - 1)) - (w * 1/np.cos(x)**2 * 1/np.cos(y)**2 * np.tan(y) * (np.sqrt(np.tan(y)**2 + np.tan(x)**2 + 1) * (2 * np.sqrt(3) * np.tan(y)**4 + 2 * np.tan(x) * np.tan(y)**3 + (2 * np.sqrt(3) * np.tan(x)**2 + 2 * np.sqrt(3)) * np.tan(y)**2 + (2 * np.tan(x)**3 + 2 * np.tan(x)) * np.tan(y)) - 2 * np.sqrt(3) * np.tan(y)**4 - 3 * np.tan(x) * np.tan(y)**3 + (-np.sqrt(3) * np.tan(x)**2 - 2 * np.sqrt(3)) * np.tan(y)**2 + (-3 * np.tan(x)**3 - 2 * np.tan(x)) * np.tan(y) + np.sqrt(3) * np.tan(x)**4)) / (2**(5 / 2) * (np.tan(y)**2 + np.tan(x)**2)**(3 / 2) * (np.tan(y)**2 + np.tan(x)**2 + 1)**(7 / 4) * (np.sqrt(np.tan(y)**2 + np.tan(x)**2 + 1) - 1)**(3 / 2))

    # d/rx val3
    dx_3 = (w * 1/np.cos(x)**2 * (-np.sqrt(3) * np.tan(x)**4 + np.sqrt(np.tan(x)**2 + np.tan(y)**2 + 1) * (2 * np.tan(y) * np.tan(x)**3 - 2 * np.sqrt(3) * np.tan(y)**2 * np.tan(x)**2 + (2 * np.tan(y)**3 + 2 * np.tan(y)) * np.tan(x) - 2 * np.sqrt(3) * np.tan(y)**4 - 2 * np.sqrt(3) * np.tan(y)**2) - 3 * np.tan(y) * np.tan(x)**3 + np.sqrt(3) * np.tan(y)**2 * np.tan(x)**2 + (-3 * np.tan(y)**3 - 2 * np.tan(y)) * np.tan(x) + 2 * np.sqrt(3) * np.tan(y)**4 + 2 * np.sqrt(3) * np.tan(y)**2)) / (2**(3 / 2) * (np.tan(x)**2 + np.tan(y)**2)**(3 / 2) * (np.tan(x)**2 + np.tan(y)**2 + 1)**(5 / 4) * np.sqrt(np.sqrt(np.tan(x)**2 + np.tan(y)**2 + 1) - 1))
    # d/rx val3
    dy_3 = (w * 1/np.cos(x)**2 * 1/np.cos(y)**2 * (np.sqrt(np.tan(y)**2 + np.tan(x)**2 + 1) * (2 * np.sqrt(3) * np.tan(y)**9 - 6 * np.tan(x) * np.tan(y)**8 + (14 * np.sqrt(3) - 2 * np.sqrt(3) * np.tan(x)**2) * np.tan(y)**7 + (-14 * np.tan(x)**3 - 35 * np.tan(x)) * np.tan(y)**6 + (-2 * 3**(5 / 2) * np.tan(x)**4 - 11 * np.sqrt(3) * np.tan(x)**2 + 20 * np.sqrt(3)) * np.tan(y)**5 + (-6 * np.tan(x)**5 - 52 * np.tan(x)**3 - 40 * np.tan(x)) * np.tan(y)**4 + (-22 * np.sqrt(3) * np.tan(x)**6 - 64 * np.sqrt(3) * np.tan(x)**4 - 22 * np.sqrt(3) * np.tan(x)**2 + 8 * np.sqrt(3)) * np.tan(y)**3 + (6 * np.tan(x)**7 + np.tan(x)**5 - 18 * np.tan(x)**3 - 16 * np.tan(x)) * np.tan(y)**2 + (-8 * np.sqrt(3) * np.tan(x)**8 - 13 * 3**(3 / 2) * np.tan(x)**6 - 14 * 3**(3 / 2) * np.tan(x)**4 - 16 * np.sqrt(3) * np.tan(x)**2) * np.tan(y) + 4 * np.tan(x)**9 + 18 * np.tan(x)**7 + 22 * np.tan(x)**5 + 8 * np.tan(x)**3) - 10 * np.sqrt(3) * np.tan(y)**9 + 24 * np.tan(x) * np.tan(y)**8 + (np.tan(y)**2 + np.tan(x)**2 + 1)**(3 / 2) * (2 * np.sqrt(3) * np.tan(y)**7 - 2 * np.tan(x) * np.tan(y)**6 + (4 * np.sqrt(3) * np.tan(x)**2 + 2 * np.sqrt(3)) * np.tan(y)**5 + (-4 * np.tan(x)**3 - 2 * np.tan(x)) * np.tan(y)**4 + (2 * np.sqrt(3) * np.tan(x)**4 + 2 * np.sqrt(3) * np.tan(x)**2) * np.tan(y)**3 + (-2 * np.tan(x)**5 - 2 * np.tan(x)**3) * np.tan(y)**2) + (-2 * 3**(3 / 2) * np.tan(x)**2 - 28 * np.sqrt(3)) * np.tan(y)**7 + (62 * np.tan(x)**3 + 58 * np.tan(x)) * np.tan(y)**6 + (14 * 3**(3 / 2) * np.tan(x)**4 + 2 * np.sqrt(3) * np.tan(x)**2 - 26 * np.sqrt(3)) * np.tan(y)**5 + (42 * np.tan(x)**5 + 88 * np.tan(x)**3 + 50 * np.tan(x)) * np.tan(y)**4 + (62 * np.sqrt(3) * np.tan(x)**6 + 88 * np.sqrt(3) * np.tan(x)**4 + 8 * 3**(3 / 2) * np.tan(x)**2 - 8 * np.sqrt(3)) * np.tan(y)**3 + (-6 * np.tan(x)**7 + 2 * np.tan(x)**5 + 24 * np.tan(x)**3 + 16 * np.tan(x)) * np.tan(y)**2 + (8 * 3**(3 / 2) * np.tan(x)**8 + 58 * np.sqrt(3) * np.tan(x)**6 + 50 * np.sqrt(3) * np.tan(x)**4 + 16 * np.sqrt(3) * np.tan(x)**2) * np.tan(y) - 10 * np.tan(x)**9 - 28 * np.tan(x)**7 - 26 * np.tan(x)**5 - 8 * np.tan(x)**3)) / (2**(5 / 2) * (np.tan(y)**2 + np.tan(x)**2)**(5 / 2) * (np.tan(y)**2 + np.tan(x)**2 + 1)**(11 / 4) * (np.sqrt(np.tan(y)**2 + np.tan(x)**2 + 1) - 1)**(3 / 2))

    return np.array([[dx_1, dy_1], [dx_2, dy_2], [dx_3, dy_3]])

N = 50
thetas = np.linspace(0, np.radians(60), N)
phis = np.linspace(0, np.radians(360), N)
# phis = np.linspace(0, 0, N)

for theta in thetas:
    for phi in phis:
        rx, ry = tp_to_rr(theta, phi)
        t1 = tendons_lengths_tp(theta, phi)
        t2 = lengths_tendons_sympy(ry, -rx)

        J1 = tendons_jacobian(theta, phi)
        J2 = tendons_jacobian_rr(rx, ry)

        print("J1: ", J1)
        print("J2: ", J2)

        # diff = t1 - t2
        # error = np.linalg.norm(diff)
        # print(theta, phi, round(error, 2))
        # print(theta, phi)
