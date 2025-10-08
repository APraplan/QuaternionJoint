import numpy as np
from time import time
from scipy.linalg import null_space
import numpy as np
from dynamixel_controller_fast import DynamixelController, BaseModel
from QuaternionJoint import QuaternionJoint
# Import from parent directory
from dynamixel_controller import Dynamixel

KP = 25.0
KI = 0
KD = 0
ANTI_WINDUP = 20
DAMPING = 0.005

class QJcontroller:
    def __init__(self, width=0.045):

        self.width = width  # m

        self.current_targets = [0, 0, 0]

        self.q_des = np.array([0.0, 0.0])
        self.q_prev = np.array([0.0, 0.0])

        self.kp = KP
        self.ki = KI
        self.kd = KD

        self.force_baseline = np.array([25, 25, 25])
        self.force_min = 10
        self.force_max = 150

        self.e_i = 0.0
        self.anti_windup_limit = ANTI_WINDUP

        self.damping = DAMPING

        self.prev_time = time()

    def _tendon_jacobian(self, q):
        
        da_dtheta = self.width * np.cos(q[0]/2) * np.cos(q[1])
        da_dphi   = -2 * self.width * np.sin(q[0]/2) * np.sin(q[1])
        db_dtheta = self.width * np.cos(q[0]/2) * np.cos(q[1] - 2*np.pi/3)
        db_dphi   = -2 * self.width * np.sin(q[0]/2) * np.sin(q[1] - 2*np.pi/3)
        dc_dtheta = self.width * np.cos(q[0]/2) * np.cos(q[1] - 4*np.pi/3)
        dc_dphi   = -2 * self.width * np.sin(q[0]/2) * np.sin(q[1] - 4*np.pi/3)

        return np.array([[da_dtheta, da_dphi], [db_dtheta, db_dphi], [dc_dtheta, dc_dphi]])
    
    def _update_time(self):
        now_time = time()
        dt = now_time - self.prev_time
        self.prev_time = now_time

        return dt
    
    def _joint_velocity(self, q, dt):

        dq = self._angle_wrap(q - self.q_prev) /dt
        self.q_prev = q
        
        return dq
    
    def _angle_wrap(self, e):
        return (e + np.pi) % (2 * np.pi) - np.pi
    
    def _anti_windup(self):
        self.e_i = np.where(self.e_i > self.anti_windup_limit, self.anti_windup_limit, self.e_i)
        self.e_i = np.where(self.e_i < -self.anti_windup_limit, -self.anti_windup_limit, self.e_i)

    def _damped_least_square_inverse(self, J, tau_des):
        JT = J.T
        tau_vec = tau_des.reshape(2,1)
        f_pinv = np.linalg.inv(J @ J.T + self.damping * np.eye(J.shape[0])) @ J @ tau_vec

        return f_pinv.flatten()
    
    def _nullspace_force_projection(self, J):
          
        N = null_space(J.T)
        if N.size == 0:
            f_proj = np.zeros_like(self.force_baseline)
        else:
            z = np.linalg.pinv(N) @ self.force_baseline
            f_proj = N @ z

        return f_proj



    def initialize(self, q):
        self.q_prev = q
        self.prev_time = time()

        return 

    def compute(self, q, q_des, qd_des):
        
        J = self._tendon_jacobian(q)

        dt = self._update_time()

        q_d = self._joint_velocity(q, dt)

        e = self._angle_wrap(q_des-q)
        e_d = qd_des - q_d
        self.e_i += e * dt

        tau_des = self.kp * e + self.kd * e_d + self.ki * self.e_i

        f_pinv = self._damped_least_square_inverse(J, tau_des)

        f_proj = self._nullspace_force_projection(J)

        f_cmd = f_pinv + f_proj

        f_cmd = np.where(f_cmd < self.force_min, self.force_min, f_cmd)
        f_cmd = np.where(f_cmd > self.force_max, self.force_max, f_cmd)

        print(f_pinv)

        return f_cmd
    
if __name__ == "__main__":

    controller = QJcontroller()
    QuaternionJoint = QuaternionJoint()

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
    start_time = time()
    print("waiting 5 sec")
    while time() < start_time + 5:
        pass

    theta, phi = QuaternionJoint.read_angles()
    q = np.array([theta, phi])
    controller.initialize(q)

    q_des = np.array([np.deg2rad(40), np.deg2rad(180)])
    qd_des = np.array([0.0, 0.0])
    

    while True:
        
        theta, phi = QuaternionJoint.read_angles()
        print("Theta, Phi:", np.rad2deg(theta), np.rad2deg(phi))
        q = np.array([theta, phi])

        f_cmd = controller.compute(q, q_des, qd_des)

        # servo.write_current(-f_cmd[0], ID=2)
        # servo.write_current(-f_cmd[1], ID=1)
        # servo.write_current(-f_cmd[2], ID=3)
        





