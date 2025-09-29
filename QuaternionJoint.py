import sys
import os

submodule_path = os.path.abspath("dynamixel-controller")
sys.path.insert(0, submodule_path)

from time import sleep
from QJ_geomerty import QJgeomerty
from Encoder import AMT23_Encoder

import numpy as np

DYNAMIXELFULLROTATION = 4096

TARGET_VID = 6790
TARGET_PID = 29987

class QuaternionJoint:
    def __init__(self, tendons_width = 75):

        self.encoders = AMT23_Encoder()
        self.encoders.connect(TARGET_VID, TARGET_PID)

        self.QJG = QJgeomerty(tendons_width)
    

    def read_angles(self):
        angle1, angle2 = self.encoders.read_angle()

        theta, phi = self.QJG.compute_theta_phi(angle1=2*angle1, angle2=2*angle2)

        phi = (phi + np.pi/2)%(2*np.pi)

        return theta, phi
    
    def read_rx_ry(self):
        angle1, angle2 = self.encoders.read_angle()
        
        rx, ry = self.QJG.compute_rx_ry(angle1=2*angle1, angle2=2*angle2)

        return rx, ry
    
if __name__ == "__main__":
    QJ = QuaternionJoint()

    # while True:
    #     rx, ry = QJ.read_rx_ry()

    #     print("Rx: ", np.round(np.rad2deg(rx), 2), " Ry: ", np.round(np.rad2deg(ry), 2))

    while True:

        theta, phi = QJ.read_angles()

        print("Theta: ", np.round(np.rad2deg(theta), 2), " Phi: ", np.round(np.rad2deg(phi), 2))

    