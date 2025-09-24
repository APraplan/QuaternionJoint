import numpy as np
from scipy.linalg import null_space
import time

from QuaternionJoint import QuaternionJoint

TARGET_VID = 6790
TARGET_PID = 29987

# ---------- Parameters ----------
width = 0.075
f_base = np.array([10.0, 10.0, 10.0])  # baseline tendon tensions
Kp = np.array([5.0, 5.0])              # proportional gains
Kd = np.array([0.1, 0.1])              # derivative gains
tau_ff = np.array([0.0, 0.0])          # optional feedforward torque
error_threshold = 1e-3                  # stopping criterion
dt = 0.01                               # control loop period (s)

# Desired joint states
theta_des = 0.15
phi_des   = 0.18
theta_dot_des = 0.0
phi_dot_des   = 0.0

# ---------- Tendon Jacobian ----------
def tendon_jacobian(theta, phi, width):
    da_dtheta = width * np.cos(theta/2) * np.cos(phi)
    da_dphi   = -2 * width * np.sin(theta/2) * np.sin(phi)
    db_dtheta = width * np.cos(theta/2) * np.cos(phi - 2*np.pi/3)
    db_dphi   = -2 * width * np.sin(theta/2) * np.sin(phi - 2*np.pi/3)
    dc_dtheta = width * np.cos(theta/2) * np.cos(phi - 4*np.pi/3)
    dc_dphi   = -2 * width * np.sin(theta/2) * np.sin(phi - 4*np.pi/3)
    J = np.array([
        [da_dtheta, da_dphi],
        [db_dtheta, db_dphi],
        [dc_dtheta, dc_dphi]
    ])
    return J

QuaternionJoint = QuaternionJoint()

# ---------- Main control loop ----------
while True:
    # Read current joint angles and velocities from sensors
    # Replace the next two lines with estimates of angles from actual encoder readings
    theta, phi = QuaternionJoint.read_angles()  # placeholder function
    print("Theta, Phi:", [theta, phi])

    # theta_dot = theta - prev_theta / dt if 'prev_theta' in locals() else 0.0
    # phi_dot = phi - prev_phi / dt if 'prev_phi' in locals() else 0.0

    # prev_theta, prev_phi = theta, phi

    q = np.array([theta, phi])
    q_dot = np.array([0, 0]) #here we should implement some cycle that does q_{i+1}-q_{i}/dt, for now lets keep it at 0
    q_des = np.array([theta_des, phi_des])
    q_dot_des = np.array([theta_dot_des, phi_dot_des])  #lets put it at 0 for now, as we are going through static poses

    # Compute errors
    e = q_des - q
    e_dot = q_dot_des - q_dot

    # Stop if error is below threshold
    if np.linalg.norm(e) < error_threshold:
        print("Target reached with error:", e)
        break

    # Desired joint torque
    tau_des = Kp * e + Kd * e_dot + tau_ff #obviously, test this by having initially only the Kp*e, then only tau_ff (nullspace component), and then a combination of the two. For the e_dot, for now lets remove it
    
    # Compute tendon forces
    J = tendon_jacobian(theta, phi, width)
    f_pinv = np.linalg.pinv(J.T) @ tau_des
    # Nullspace projection of baseline tension

    N = null_space(J.T)
    if N.size == 0:
        f_proj = np.zeros_like(f_base)
    else:
        z = np.linalg.pinv(N) @ f_base
        f_proj = N @ z
    f_cmd = f_pinv + f_proj

    # Send forces to tendon actuators
    # send_tendon_forces(f_cmd)  # placeholder function
    print("forces:", f_cmd)
    time.sleep(dt)  # maintain control loop period