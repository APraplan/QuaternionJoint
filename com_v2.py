import numpy as np
import time
from scipy.linalg import null_space
import numpy as np
from dynamixel_controller_fast import DynamixelController, BaseModel
from QuaternionJoint import QuaternionJoint
import matplotlib.pyplot as plt

lqj = 150
alpha = np.radians(20)
lm1 = 200

beta = np.radians(30)
lp = 400

pivot_angle = np.radians(0)
m2_angle = np.radians(5)
gamma = np.radians(120)+m2_angle-pivot_angle
lm2 = 170

if __name__ == "__main__":

    QuaternionJoint = QuaternionJoint()
    rx, ry = QuaternionJoint.read_rx_ry()

    q = np.array([rx, ry])

    fig1 = plt.figure()
    ax1 = fig1.add_subplot(111, projection='3d')
    

    while True:
        
        # --- Read current state ---
        rx, ry = QuaternionJoint.read_rx_ry()
        ry = -ry
        q = np.array([rx, ry])

        ax1.clear()

        ax1.quiver(0, 0, 0, 0.5, 0, 0, color='red', label='x_axis')
        ax1.quiver(0, 0, 0, 0, 0.5, 0, color='green', label='y_axis')
        ax1.quiver(0, 0, 0, 0, 0, 0.5, color='blue', label='z_axis')

        # --- M1 ---

        h_qj = lqj*np.cos(rx)*np.cos(ry)
        h_m1 = lm2*np.cos(2*rx)*np.cos(2*ry+alpha)

        h1 = lqj*np.cos(rx)*np.cos(ry) + lm2*np.cos(2*rx)*np.cos(2*ry+alpha)

        # ∂h1/∂rx
        dh1_drx = -lqj*np.sin(rx)*np.cos(ry) - 2*lm2*np.sin(2*rx)*np.cos(2*ry + alpha)

        # ∂h1/∂ry
        dh1_dry = -lqj*np.cos(rx)*np.sin(ry) - 2*lm2*np.cos(2*rx)*np.sin(2*ry + alpha)


        # --- M2 ---
        
        h_qj = lqj*np.cos(rx)*np.cos(ry)
        h_p = lp*np.cos(2*rx)*np.cos(2*ry+beta)
        h_m2 = lm2*np.cos(2*rx)*np.cos(2*ry+gamma)

        h2 = lqj*np.cos(rx)*np.cos(ry) + np.cos(2*rx)*(lp*np.cos(2*ry+beta) +lm2*np.cos(2*ry+gamma))

        # ∂h2/∂rx
        dh2_drx = -lqj*np.sin(rx)*np.cos(ry) - 2*np.sin(2*rx)*(lp*np.cos(2*ry + beta) + lm2*np.cos(2*ry + gamma))

        # ∂h2/∂ry
        dh2_dry = -lqj*np.cos(rx)*np.sin(ry) - 2*np.cos(2*rx)*(lp*np.sin(2*ry + beta) + lm2*np.sin(2*ry + gamma))


        # Axes setup
        ax1.set_xlim([-3.5, 3.5])
        ax1.set_ylim([-3.5, 3.5])
        ax1.set_zlim([-3.5, 3.5])
        ax1.set_xlabel('X')
        ax1.set_ylabel('Y')
        ax1.set_zlabel('Z')
        ax1.set_title('Vectors v1, v2, and h')
        ax1.legend()

        plt.tight_layout()
        plt.draw()
        plt.pause(0.0001) 



