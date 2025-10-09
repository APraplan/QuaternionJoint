import numpy as np
from time import time
from scipy.linalg import null_space
import numpy as np
from dynamixel_controller_fast import DynamixelController, BaseModel
from QuaternionJoint import QuaternionJoint
# Import from parent directory
from dynamixel_controller import Dynamixel

KP = 45.0
KI = 0
KD = 0
ANTI_WINDUP = 20
DAMPING = 0.001

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
        self.force_min = 25
        self.force_max = 150

        self.e_i = 0.0
        self.anti_windup_limit = ANTI_WINDUP

        self.damping = DAMPING

        self.prev_time = time()

    def _tendons_jacobian(self, q, width=0.045):
        """
        Compute the 3×2 Jacobian matrix of (val1, val2, val3)
        with respect to (rx, ry), safely handling rx=ry=0.

        Parameters
        ----------
        q : array-like of shape (2,)
            [rx, ry] rotations in radians.
        width : float
            Scaling factor used in val expressions.

        Returns
        -------
        J : ndarray of shape (3, 2)
            Jacobian matrix [[∂val_i/∂rx, ∂val_i/∂ry], i=1..3]
        """

        rx = float(q[0])
        ry = float(q[1])

        tx = np.tan(rx)
        ty = np.tan(ry)

        A = np.sqrt(1 + tx**2 + ty**2)
        B = np.sqrt(tx**2 + ty**2)

        # ---- Handle near-zero B safely ----
        eps = 1e-9
        if B < eps:
            # At rx=ry=0, we can derive the limit analytically.
            # Using small-angle approximations: tan(x)≈x, A≈1
            # The common factor C ≈ width * sqrt( (A - 1)/(A) ) → 0 linearly.
            # Jacobian tends to 0 in most components except proportional terms.
            return np.array([
                [0.0,  2 * width * 0.5],          # derivative w.r.t ry dominates for val1
                [-np.sqrt(3) * width / 2, -width], # approximate pattern
                [ np.sqrt(3) * width / 2, -width]
            ])

        # -----------------------------------

        C = 2 * width * np.sqrt((A - 1) / (2 * A))

        # Common partials for A, B
        dA_drx = (tx / A) / np.cos(rx)**2
        dA_dry = (ty / A) / np.cos(ry)**2
        dB_drx = (tx / B) / np.cos(rx)**2
        dB_dry = (ty / B) / np.cos(ry)**2

        # Derivative of C wrt A
        dC_dA = width * (A + 1) / (A**2 * np.sqrt(2 * A * (A - 1)))
        dC_drx = dC_dA * dA_drx
        dC_dry = dC_dA * dA_dry

        # n1, n2, n3 and their partials wrt rx, ry
        n1 = ty / B
        n2 = (-ty - np.sqrt(3) * tx) / (2 * B)
        n3 = (-ty + np.sqrt(3) * tx) / (2 * B)

        # partials of n_i
        dn1_drx = -ty * dB_drx / B**2
        dn1_dry = (1 / np.cos(ry)**2) / B - ty * dB_dry / B**2

        dn2_drx = (-np.sqrt(3) / (2 * np.cos(rx)**2)) / B - (-ty - np.sqrt(3)*tx) * dB_drx / (2 * B**2)
        dn2_dry = (-1 / (2 * np.cos(ry)**2)) / B - (-ty - np.sqrt(3)*tx) * dB_dry / (2 * B**2)

        dn3_drx = ( np.sqrt(3) / (2 * np.cos(rx)**2)) / B - (-ty + np.sqrt(3)*tx) * dB_drx / (2 * B**2)
        dn3_dry = (-1 / (2 * np.cos(ry)**2)) / B - (-ty + np.sqrt(3)*tx) * dB_dry / (2 * B**2)

        # Jacobian rows for val1, val2, val3
        J = np.zeros((3, 2))
        for i, (ni, dnrx, dnry) in enumerate([
            (n1, dn1_drx, dn1_dry),
            (n2, dn2_drx, dn2_dry),
            (n3, dn3_drx, dn3_dry)
        ]):
            J[i, 0] = dC_drx * ni + C * dnrx
            J[i, 1] = dC_dry * ni + C * dnry

        return J

    
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
        
        J = self._tendons_jacobian(q)

        dt = self._update_time()

        q_d = self._joint_velocity(q, dt)

        e = q_des-q
        e_d = qd_des - q_d
        self.e_i += e * dt

        tau_des = self.kp * e + self.kd * e_d + self.ki * self.e_i

        f_pinv = self._damped_least_square_inverse(J, tau_des)

        f_proj = self._nullspace_force_projection(J)

        f_cmd = f_pinv + f_proj

        f_cmd = np.where(f_cmd < self.force_min, self.force_min, f_cmd)
        f_cmd = np.where(f_cmd > self.force_max, self.force_max, f_cmd)

        # print(f_cmd)

        return f_cmd
    
def tendon_lengths_test(rx, ry):
    width = 45  # mm

    # rx, ry are in radians
    A = np.sqrt(1 + np.tan(rx)**2 + np.tan(ry)**2)
    B = np.sqrt(np.tan(rx)**2 + np.tan(ry)**2)

    # 1) sin(theta/2) * cos(phi)
    val1 = 2 * width * np.sqrt((A - 1) / (2 * A)) * (np.tan(ry) / B)

    # 2) sin(theta/2) * cos(phi - 2/3 * pi)
    val2 = 2 * width * np.sqrt((A - 1) / (2 * A)) * ((-np.tan(ry) - np.sqrt(3) * np.tan(rx)) / (2 * B))

    # 3) sin(theta/2) * cos(phi - 4/3 * pi)
    val3 = 2 * width * np.sqrt((A - 1) / (2 * A)) * ((-np.tan(ry) + np.sqrt(3) * np.tan(rx)) / (2 * B))

    return np.array([val1, val2, val3])
    
if __name__ == "__main__":

    controller = QJcontroller()
    QuaternionJoint = QuaternionJoint()


    port = "/dev/ttyUSB2"
    dxl_ids = [2, 1, 3]
    bdrt = 57600
    f_base = np.array([25,25,25])

    servo = Dynamixel(ID=dxl_ids, descriptive_device_name="XM430 test motor",
                      series_name=["xm", "xm", "xm"], baudrate=bdrt, port_name=port)

    servo.begin_communication()
    servo.set_operating_mode("current", ID="all")
    servo.write_current(10, ID="all")
    start_time = time()
    print("waiting 3 sec")
    while time() < start_time + 3:
        pass

    rx, ry = QuaternionJoint.read_rx_ry()
    q = np.array([rx, ry])
    controller.initialize(q)

    q_des = np.array([np.deg2rad(0), np.deg2rad(30)])
    qd_des = np.array([0.0, 0.0])
    

    while True:

        start_time = time()
        
        rx, ry = QuaternionJoint.read_rx_ry()

        # print("rx, ry:", np.rad2deg(rx), np.rad2deg(ry))
        q = np.array([rx, ry])

        try:
            f_cmd = controller.compute(q, q_des, qd_des)

            # print("Tendon lengths (m):", -tendon_lengths_test(rx, ry))

            servo.write_current(f_cmd[0], ID=2)
            servo.write_current(f_cmd[1], ID=1)
            servo.write_current(f_cmd[2], ID=3)
        except Exception as e:
            print(f"An error occurred: {e}")
            continue

        print("rx, ry:", np.rad2deg(rx), np.rad2deg(ry))
        
        





