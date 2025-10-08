import numpy as np
from scipy.linalg import null_space
import time
from dynamixel_controller_fast import DynamixelController, BaseModel
from QuaternionJoint import QuaternionJoint
# Import from parent directory
from dynamixel_controller import Dynamixel

PULLEYDIAMETER = 20
DYNAMIXELFULLROTATION = 4096

TARGET_VID = 6790
TARGET_PID = 29987

# ---------- Parameters ----------
width = 0.084
error_threshold = 1e-3                  # stopping criterion
dt = 0.01                               # control loop period (s)

# Desired joint states
theta_des = 0.15
phi_des   = 0.18
theta_dot_des = 0.0
phi_dot_des   = 0.0


def mm2pos(d_mm):
    return d_mm * DYNAMIXELFULLROTATION / PULLEYDIAMETER / np.pi


# ---------- Tendon Jacobian ----------
def tendon_jacobian(theta, phi):
    width = 0.084
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

def numerical_jacobian(theta, phi, width=0.084, eps=1e-6):
        f0 = tendon_lengths(theta, phi, width)
        jac = np.zeros((3, 2))  # 3 tendon lengths, 2 variables (theta, phi)

        # Partial derivative w.r.t theta
        f_theta = tendon_lengths(theta + eps, phi, width)
        jac[:, 0] = (f_theta - f0) / eps

        # Partial derivative w.r.t phi
        f_phi = tendon_lengths(theta, phi + eps, width)
        jac[:, 1] = (f_phi - f0) / eps

        return jac



    # ---------- Tendon length mapping ----------
def tendon_lengths(theta, phi, width):
    phi=phi
    da = 2*width*np.sin(theta/2)*np.cos(phi)
    db = 2*width*np.sin(theta/2)*np.cos(phi - 2*np.pi/3)
    dc = 2*width*np.sin(theta/2)*np.cos(phi - 4*np.pi/3)
    return -np.array([da, db, dc])

QuaternionJoint = QuaternionJoint()

TEST = 12



if TEST == -2:
    t = tendon_lengths(0.2, np.deg2rad(120), 84)
    print(t)

if TEST == -1:
    port = "/dev/ttyUSB1"
    dxl_ids = [2, 3, 1]
    bdrt = 57600

    servo = Dynamixel(ID=dxl_ids, descriptive_device_name="XM430 test motor",
                      series_name=["xm", "xm", "xm"], baudrate=bdrt, port_name=port)

    servo.begin_communication()
    servo.set_operating_mode("current", ID="all")
    servo.write_current(-10, ID="all")
    time.sleep(10)
    servo.set_operating_mode("extended position", ID="all")
    goal_pos=[np.deg2rad(40),np.deg2rad(0)]
    t=tendon_lengths(goal_pos[0],goal_pos[1],84)
    theta, phi = QuaternionJoint.read_angles()
    print("Theta, Phi measured:", np.rad2deg(theta), np.rad2deg(phi))


    rec_pos=servo.read_position( ID="all")
    motor_command=rec_pos+mm2pos(t)

    servo.write_position(motor_command[0], ID=2)
    servo.write_position(motor_command[1], ID=1)
    servo.write_position(motor_command[2], ID=3)

    time.sleep(5)
    theta, phi = QuaternionJoint.read_angles()
    print("Theta, Phi measured after movement:", np.rad2deg(theta), np.rad2deg(phi))

    goal_pos = [np.deg2rad(40), np.deg2rad(90)]
    t = tendon_lengths(goal_pos[0], goal_pos[1], 84)
    theta, phi = QuaternionJoint.read_angles()
    print("Theta, Phi measured:", np.rad2deg(theta), np.rad2deg(phi))
    motor_command = rec_pos + mm2pos(t)

    servo.write_position(motor_command[0], ID=2)
    servo.write_position(motor_command[1], ID=1)
    servo.write_position(motor_command[2], ID=3)

    time.sleep(5)
    theta, phi = QuaternionJoint.read_angles()
    print("Theta, Phi measured after movement:", np.rad2deg(theta), np.rad2deg(phi))

    goal_pos = [np.deg2rad(40), np.deg2rad(180)];
    t = tendon_lengths(goal_pos[0], goal_pos[1], 84)
    theta, phi = QuaternionJoint.read_angles()
    print("Theta, Phi measured:", np.rad2deg(theta), np.rad2deg(phi))
    motor_command = rec_pos + mm2pos(t)

    servo.write_position(motor_command[0], ID=2)
    servo.write_position(motor_command[1], ID=1)
    servo.write_position(motor_command[2], ID=3)

    time.sleep(5)
    theta, phi = QuaternionJoint.read_angles()
    print("Theta, Phi measured after movement:", np.rad2deg(theta), np.rad2deg(phi))




elif TEST == 0:
    while True:
        # ---------- Test Jacobian with delta joint angles ----------
        # here replace it in a way that you put theta and phi from encoder's readings
        theta, phi = QuaternionJoint.read_angles()
        print("Theta, Phi:", np.rad2deg(theta), np.rad2deg(phi))
elif TEST == 1:
    while True:
        # ---------- Test Jacobian with delta joint angles ----------
        # here replace it in a way that you put theta and phi from encoder's readings
        theta, phi = QuaternionJoint.read_angles()
        print("Theta, Phi:", np.rad2deg(theta), np.rad2deg(phi))

        # small joint motion (edit this to check first delt theta and then delta phi)
        delta_q = np.array([0.5, 0])  # [delta_theta, delta_phi]
        # compute Jacobian
        J = tendon_jacobian(theta, phi)
        # predict tendon length changes
        delta_l = J @ delta_q


        print("Predicted tendon length changes:", delta_l)

if TEST == 1.5:
    q_des = np.array([0.2, 2])
    while True:
        # ---------- Test Jacobian with delta joint angles ----------
        # here replace it in a way that you put theta and phi from encoder's readings
        theta, phi = QuaternionJoint.read_angles()
        print("GOAL:", np.rad2deg(q_des))
        print("Theta, Phi:", np.rad2deg(theta), np.rad2deg(phi))
        q = np.array([theta, phi])

        # small joint motion (edit this to check first delt theta and then delta phi)
        e = q_des - q
        print("error:", np.rad2deg(e))
        # compute Jacobian
        J = numerical_jacobian(theta, phi)
        # predict tendon length changes
        delta_l = J @ e
        print("Predicted tendon length changes:", delta_l)
        time.sleep(3)




if TEST == 2:
    port = "COM6"
    dxl_ids = [2, 3, 1]
    bdrt = 57600

    servo = Dynamixel(ID=dxl_ids, descriptive_device_name="XM430 test motor",
                      series_name=["xm", "xm", "xm"], baudrate=bdrt, port_name=port)

    servo.begin_communication()
    servo.set_operating_mode("current", ID="all")
    servo.write_current(-10, ID="all")
    time.sleep(5)
    servo.set_operating_mode("extended position", ID="all")

    for i in range(150):

        theta, phi = QuaternionJoint.read_angles()

        # small joint motion (edit this to check first delt theta and then delta phi)
        delta_q = np.array([0, -0.5])  # [delta_theta, delta_phi]
        # compute Jacobian
        J = tendon_jacobian(theta, phi)
        # predict tendon length changes
        delta_l =  J @ delta_q

        print("Theta, Phi:", np.rad2deg(theta), np.rad2deg(phi))
        scaled_delta_l=10*mm2pos(delta_l)
        print("Predicted tendon length changes:", scaled_delta_l)
        curr_pos = servo.read_position(ID="all")
        # print("Curr_pos:", curr_pos)
        motor_command = curr_pos + scaled_delta_l

        # print("Motor command:", motor_command)
        servo.write_position(motor_command[0], ID=2)
        servo.write_position(motor_command[1], ID=1)
        servo.write_position(motor_command[2], ID=3)

if TEST == 3:
    port = "COM6"
    dxl_ids = [2, 3, 1]
    bdrt = 57600

    servo = Dynamixel(ID=dxl_ids, descriptive_device_name="XM430 test motor",
                      series_name=["xm", "xm","xm"], baudrate=bdrt, port_name=port)

    servo.begin_communication()
    servo.set_operating_mode("current", ID="all")
    servo.write_current(-10, ID="all")
    time.sleep(5)
    servo.set_operating_mode("extended position", ID="all")

    #curr_pos=servo.read_position(ID="all")
    #print("Curr_pos:",curr_pos)

    #Initialize control goal
    q_des = np.array([np.deg2rad(30), np.deg2rad(180)])
    error_threshold = 1e-3

    while True:
        # ---------- Test closed loop with delta joint angles ----------
        # here replace it in a way that you put theta and phi from encoder's readings
        theta, phi = QuaternionJoint.read_angles()
        print("Theta, Phi:", np.rad2deg(theta), np.rad2deg(phi))

        q=np.array([theta,phi])
        J = tendon_jacobian(theta, phi)

        # Compute errors
        e =  q_des-q
        #e_dot = q_dot_des - q_dot
        print("error", e)
        # Stop if error is below threshold
        if np.linalg.norm(e) < error_threshold:
            print("Target reached with error:", e)
            break

        k_gain=10000
        # predict tendon length changes
        delta_l = J @ e
        print("tendon update:", delta_l)

        #print("Error:", e)
        curr_pos = servo.read_position(ID="all")
        #print("Curr_pos:", curr_pos)
        motor_command=curr_pos+k_gain*delta_l
        #print("Motor command:", motor_command)
        servo.write_position(motor_command[0], ID=2)
        servo.write_position(motor_command[1], ID=1)
        servo.write_position(motor_command[2], ID=3)


elif TEST == 4:

    theta_des = 0.0
    phi_des   = 0.0
    theta_dot_des = 0.0
    phi_dot_des   = 0.0

    # ---------- Main control loop ----------
    while True:
        # Read current joint angles and velocities from sensors
        # Replace the next two lines with estimates of angles from actual encoder readings
        theta, phi = QuaternionJoint.read_angles()  # placeholder function
        print("Theta, Phi:", theta, phi)

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

elif TEST==6:
    port = "COM6"
    dxl_ids = [2, 3, 1]
    bdrt = 57600

    servo = Dynamixel(ID=dxl_ids, descriptive_device_name="XM430 test motor",
                      series_name=["xm", "xm", "xm"], baudrate=bdrt, port_name=port)

    servo.begin_communication()
    servo.set_operating_mode("current", ID="all")
    servo.write_current(-10, ID="all")
    time.sleep(7)
    servo.set_operating_mode("velocity", ID="all")

    # user/hardware params (fill these)
    spool_radius = 0.01  # meters (example)
    gear_ratio = 1.0  # motor output revolutions per tendon wrap revolution (if any)
    ticks_per_rev = 4096  # encoder ticks per motor revolution (if using ticks)
    max_motor_vel_ticks = 2000  # max allowed velocity (ticks/sec) — safety clip
    k_gain = 20  # Kp for joint velocity (tune)


    # Main loop snippet
    q_des = np.array([0.2, 3.0])  # rad
    error_threshold = 1e-3

    while True:
            theta, phi = QuaternionJoint.read_angles()  # radians
            q = np.array([theta, phi])
            e = q_des - q

            if np.linalg.norm(e) < error_threshold:
                print("Target reached with error:", e)
                break

            # 1) joint velocity command (resolved-rate control)
            q_dot = k_gain * e  # rad/s (tune k_gain)

            # 2) tendon length velocity from Jacobian
            J = numerical_jacobian(theta, phi)  # (3x2)
            l_dot = J @ q_dot  # (3,) meters/sec

            # 3) redundancy: add nullspace velocity to maintain/preload tendon tension
            # compute pseudoinverse once per loop
            #J_pinv = np.linalg.pinv(J)  # (2x3)
            #N = np.eye(3) - J @ J_pinv  # projector to nullspace (3x3)

            # measure current tendon lengths from motor positions if possible:
            # Example: if servo.read_position returns ticks for motors in order [m2, m3, m1]
            curr_pos_ticks = servo.read_position(ID="all")  # shape (3,) - fill based on your API
            # convert ticks -> tendon length (meters)


            # compute nullspace command to push tendon lengths toward nominal (simple PI/P control)
            #v_null = k_null * (l_nom - l_current)  # meters/sec (3,)


            # 4) convert tendon velocity -> motor velocity units your servo expects
            motor_vel_ticks =-k_gain* l_dot

            # 6) send velocity commands to motors
            # NOTE: adapt these calls to your servo API. Example: servo.write_velocity(velocity, ID=2)
            # Also be careful with ordering: you previously wrote motors in order [2,3,1] for three tendons.
            print("velocity update:", motor_vel_ticks)
            print("Theta, Phi:", np.rad2deg(theta), np.rad2deg(phi))

            servo.write_velocity(motor_vel_ticks[0], ID=2)
            servo.write_velocity(motor_vel_ticks[1], ID=1)
            servo.write_velocity(motor_vel_ticks[2], ID=3)

elif TEST == 10:
        port = "COM6"
        dxl_ids = [2, 3, 1]
        bdrt = 57600

        servo = Dynamixel(ID=dxl_ids, descriptive_device_name="XM430 test motor",
                          series_name=["xm", "xm", "xm"], baudrate=bdrt, port_name=port)

        servo.begin_communication()
        servo.set_operating_mode("current", ID="all")
        servo.write_current(-10, ID="all")
        time.sleep(5)
        servo.set_operating_mode("velocity", ID="all")

        # curr_pos=servo.read_position(ID="all")
        # print("Curr_pos:",curr_pos)

        # Initialize control goal
        q_des = np.array([np.deg2rad(30), np.deg2rad(180)])
        error_threshold = 1e-3

        while True:
            # ---------- Test closed loop with delta joint angles ----------
            # here replace it in a way that you put theta and phi from encoder's readings
            theta, phi = QuaternionJoint.read_angles()
            print("Theta, Phi:", np.rad2deg(theta), np.rad2deg(phi))

            q = np.array([theta, phi])
            J = tendon_jacobian(theta, phi)

            # Compute errors
            e = q_des - q
            # e_dot = q_dot_des - q_dot
            print("error", e)
            # Stop if error is below threshold
            if np.linalg.norm(e) < error_threshold:
                print("Target reached with error:", e)
                break

            k_gain = 5000
            # predict tendon length changes
            delta_l = J @ e
            print("tendon update:", delta_l)

            # print("Error:", e)
            curr_pos = servo.read_position(ID="all")
            # print("Curr_pos:", curr_pos)
            motor_command = k_gain*delta_l
            # print("Motor command:", motor_command)
            servo.write_velocity(motor_command[0], ID=2)
            servo.write_velocity(motor_command[1], ID=1)
            servo.write_velocity(motor_command[2], ID=3)



elif TEST == 11:
        port = "COM6"
        dxl_ids = [2, 3, 1]
        bdrt = 57600
        f_base = np.array([-20, -20, -20])

        servo = Dynamixel(ID=dxl_ids, descriptive_device_name="XM430 test motor",
                          series_name=["xm", "xm", "xm"], baudrate=bdrt, port_name=port)

        servo.begin_communication()
        servo.set_operating_mode("current", ID="all")
        servo.write_current(-10, ID="all")
        time.sleep(15)
        servo.set_operating_mode("current", ID="all")

        # curr_pos=servo.read_position(ID="all")
        # print("Curr_pos:",curr_pos)

        # Initialize control goal
        q_des = np.array([np.deg2rad(40), np.deg2rad(20)])
        error_threshold = 1e-3

        while True:
            # ---------- Test closed loop with delta joint angles ----------
            # here replace it in a way that you put theta and phi from encoder's readings
            theta, phi = QuaternionJoint.read_angles()
            print("Theta, Phi:", np.rad2deg(theta), np.rad2deg(phi))

            q = np.array([theta, phi])
            J = tendon_jacobian(theta, phi)

            # Compute errors
            e = q_des - q


            print("error", e)
            # Stop if error is below threshold
            if np.linalg.norm(e) < error_threshold:
                print("Target reached with error:", e)
                break

            Kp = 10
            tau_des = Kp * e
            # predict tendon length changes
            f_pinv = np.linalg.pinv(J.T) @ tau_des
            # Nullspace projection of baseline tension

            N = null_space(J.T)
            if N.size == 0:
                f_proj = np.zeros_like(f_base)
            else:
                z = np.linalg.pinv(N) @ f_base
                f_proj = N @ z
            f_cmd = f_pinv + f_proj
            #f_cmd = f_pinv

            #f_cmd = f_proj

            # print("Motor command:", motor_command)
            servo.write_current(f_cmd[0], ID=2)
            servo.write_current(f_cmd[1], ID=1)
            servo.write_current(f_cmd[2], ID=3)


elif TEST == 12:
    import time

    def angle_wrap(e):
        return (e + np.pi) % (2 * np.pi) - np.pi

    port = "/dev/ttyUSB1"
    dxl_ids = [2, 1, 3]
    bdrt = 57600
    f_base = np.array([-25,-25,-25])

    servo = Dynamixel(ID=dxl_ids, descriptive_device_name="XM430 test motor",
                      series_name=["xm", "xm", "xm"], baudrate=bdrt, port_name=port)

    servo.begin_communication()
    servo.set_operating_mode("current", ID="all")
    servo.write_current(-10, ID="all")
    time.sleep(10)
    servo.set_operating_mode("current", ID="all")

    # Initialize control goal
    q_des = np.array([np.deg2rad(40), np.deg2rad(180)])
    qd_des = np.array([0, 0])
    error_threshold = 1e-3

    # Initialize timing and storage
    q_prev = None
    t_prev = time.time()
    integral_error = np.zeros_like(q_des)
    Kp = 40
    Kd = 1
    Ki = 1

    while True:
        # ---------- Read encoder ----------
        theta, phi = QuaternionJoint.read_angles()
        print("Theta, Phi:", np.rad2deg(theta), np.rad2deg(phi))

        q = np.array([theta, phi])

        # Jacobian
        J = tendon_jacobian(theta, phi)

        # Compute time step
        t_now = time.time()
        dt = t_now - t_prev
        t_prev = t_now

        # Compute joint velocity (qd)
        if q_prev is None:
            qd = np.zeros_like(q)
        else:
            dq = angle_wrap(q - q_prev)
            qd = dq / dt
        q_prev = q



        # Compute errors
        e = angle_wrap(q_des - q)
        e_dot = qd_des - qd
        integral_error += e * dt  # accumulate error
        ## Really needed ???????????

        # Stop if error is below threshold
        if np.linalg.norm(e) < error_threshold:
            print("Target reached with error:", e)
            break

        # -------- PID Control law --------
        tau_des = Kp * e + Kd * e_dot + Ki * integral_error
        #optional shit: this is to don t consider forces projected on phi for small theta angles
        # if abs(theta) < np.deg2rad(5):
        #     tau_des[1] = 0

        # Traditional pseudiìoinverse approach (good but doesn t work around singularity theta=0)
        #f_pinv = np.linalg.pinv(J.T) @ tau_des  #this is the normal approach

        #instead,this is with damped least squares approach to compensate singularity around theta=0
        damping = 0.005
        JT = J.T
        tau_vec = tau_des.reshape(2,1)
        # Damped least squares
        f_pinv = np.linalg.inv(J @ J.T + damping * np.eye(J.shape[0])) @ J @ tau_vec
        f_pinv = f_pinv.flatten()

        # Nullspace projection of baseline tension
        N = null_space(J.T)
        if N.size == 0:
            f_proj = np.zeros_like(f_base)
        else:
            z = np.linalg.pinv(N) @ f_base
            f_proj = N @ z

        print(f_pinv)

        f_cmd = f_pinv + f_proj
        #tendons can only pull
        f_cmd = np.where(f_cmd < 10, 10, f_cmd)

        # Send currents
        ## Negative command == Unspool
        # servo.write_current(-f_cmd[0], ID=2)
        # servo.write_current(-f_cmd[1], ID=1)
        # servo.write_current(-f_cmd[2], ID=3)
