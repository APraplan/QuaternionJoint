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

        theta, phi = self.QJG.compute_theta_phi(angle1=2*angle1, angle1_pos=np.deg2rad(60), angle2=2*angle2, angle2_pos=np.deg2rad(180))

        phi = (phi + np.pi/2)%(2*np.pi)

        return theta, phi

    