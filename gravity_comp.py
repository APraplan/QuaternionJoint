import numpy as np
import time
from scipy.linalg import null_space
import numpy as np
from dynamixel_controller_fast import DynamixelController, BaseModel
from QuaternionJoint import QuaternionJoint
import matplotlib.pyplot as plt

qj_h = 1.50
alpha = np.radians(20)
lm1 = 2.0

beta = np.radians(30)
lp = 4

pivot_angle = np.radians(0)
m2_angle = np.radians(5)
gamma = np.radians(120)+m2_angle-pivot_angle
lm2 = 1.7

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

        vrx = np.array([0, np.cos(rx), np.sin(rx)], dtype=float)
        vry = np.array([np.cos(ry), 0, np.sin(ry)], dtype=float)
        ax1.quiver(0, 0, 0, vrx[0], vrx[1], vrx[2], color='orange', label='vrx')
        ax1.quiver(0, 0, 0, vry[0], vry[1], vry[2], color='orange', label='vry')

        n1 = np.array([-np.cos(rx)*np.sin(ry)*qj_h/np.sqrt(np.cos(rx)**2+np.sin(rx)**2*np.cos(ry)**2), -np.sin(rx)*np.cos(ry)*qj_h/np.sqrt(np.cos(rx)**2+np.sin(rx)**2*np.cos(ry)**2), np.cos(rx)*np.cos(ry)*qj_h/np.sqrt(np.cos(rx)**2+np.sin(rx)**2*np.cos(ry)**2)])

        ax1.quiver(0, 0, 0, n1[0], n1[1], n1[2], color='lightgreen', label='n1')

        n2x = (-np.cos(2*rx)*np.sin(2*ry)*np.cos(alpha) + np.cos(2*ry)*np.sin(alpha))
        n2y = (-np.sin(2*rx)*np.cos(2*ry)*np.cos(alpha) + np.sin(2*rx)*np.cos(2*rx)*np.sin(2*ry)*np.sin(alpha))
        n2z = (np.cos(2*rx)*np.cos(2*ry)*np.cos(alpha) + np.cos(2*rx)**2*np.sin(2*ry)*np.sin(alpha))

        n2 = np.array([n2x, n2y, n2z])
        n2 = n2/np.sqrt(np.cos(rx)**2+np.sin(rx)**2*np.cos(ry)**2)*lm1

        # print(np.linalg.norm(n1))
        # print(np.linalg.norm(n2))

        ax1.quiver(0, 0, 0, n2[0], n2[1], n2[2], color='green', label='n2')

        vm1 = n1+n2

        ax1.quiver(0, 0, 0, vm1[0], vm1[1], vm1[2], color='blue', label='vm1')

        vm1z = np.cos(rx)*np.cos(ry)*qj_h/np.sqrt(np.cos(rx)**2+np.sin(rx)**2*np.cos(ry)**2)+(np.cos(2*rx)*np.cos(2*ry)*np.cos(alpha) + np.cos(2*rx)**2*np.sin(2*ry)*np.sin(alpha))/np.sqrt(np.cos(rx)**2+np.sin(rx)**2*np.cos(ry)**2)*lm1


        # --- M2 ---
        n3x = (-np.cos(2*rx)*np.sin(2*ry)*np.cos(beta) + np.cos(2*ry)*np.sin(beta))
        n3y = (-np.sin(2*rx)*np.cos(2*ry)*np.cos(beta) + np.sin(2*rx)*np.cos(2*rx)*np.sin(2*ry)*np.sin(beta))
        n3z = (np.cos(2*rx)*np.cos(2*ry)*np.cos(beta) + np.cos(2*rx)**2*np.sin(2*ry)*np.sin(beta))
        
        n3 = np.array([n3x, n3y, n3z])
        # n3 = n3/np.sqrt(np.cos(2*ry)**2+np.cos(2*rx)**2*np.sin(2*ry)**2)*lp
        n3 = n3/np.sqrt(np.cos(2*rx)**2+np.sin(2*rx)**2*np.cos(2*ry)**2)*lp     
        

        ax1.quiver(0, 0, 0, n3[0], n3[1], n3[2], color='yellow', label='n3')

        n4x = (-np.cos(2*rx)*np.sin(2*ry)*np.cos(gamma) + np.cos(2*ry)*np.sin(gamma))
        n4y = (-np.sin(2*rx)*np.cos(2*ry)*np.cos(gamma) + np.sin(2*rx)*np.cos(2*rx)*np.sin(2*ry)*np.sin(gamma))
        n4z = (np.cos(2*rx)*np.cos(2*ry)*np.cos(gamma) + np.cos(2*rx)**2*np.sin(2*ry)*np.sin(gamma))
        
        n4 = np.array([n4x, n4y, n4z])
        n4 = n4/np.sqrt(np.cos(2*rx)**2+np.sin(2*rx)**2*np.cos(2*ry)**2)*lm2

        ax1.quiver(0, 0, 0, n4[0], n4[1], n4[2], color='red', label='n4')

        # ax1.quiver(n3[0], n3[1], n3[2], n3[0]+n4[0], n3[1]+n4[1], n3[2]+n4[2], color='red', label='n4')

        vm2 = n3 + n4
        ax1.quiver(0, 0, 0, vm2[0], vm2[1], vm2[2], color='orange', label='vm2')

        vm2z = [np.cos(2*rx)*np.cos(2*ry)*(lp*np.cos(beta) + lm2*np.cos(gamma))+ np.cos(2*rx)**2*np.sin(2*ry)*(lp*np.sin(beta) + lm2*np.sin(gamma))]/ np.sqrt(np.cos(2*rx)**2 + np.sin(2*rx)**2*np.cos(2*ry)**2)

        # print(np.linalg.norm(n3))
        # print(np.linalg.norm(n4))


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



