import sys
import os

submodule_path = os.path.abspath("dynamixel-controller")
sys.path.insert(0, submodule_path)

from time import sleep
from dynamixel_controller import Dynamixel
from QJ_geomerty import QJgeomerty

import keyboard
import time
import numpy as np

PULLEYDIAMETER = 20
DYNAMIXELFULLROTATION = 4096

CURRENT_LIMIT = 100


class QuaternionJointK:
    def __init__(self, dxl_ids = [1, 2, 3], bdrt = 57600, tendons_width = 75):
        

        self.Servo1 = Dynamixel(dxl_ids[0], descriptive_device_name="XM430 test motor", 
                series_name="xm", baudrate=bdrt, port_name="/dev/ttyUSB0")
        self.Servo1.begin_communication()
        self.Servo1.set_operating_mode("extended position")

        self.Servo2 = Dynamixel(dxl_ids[1], descriptive_device_name="XM430 test motor", 
                series_name="xm", baudrate=bdrt, port_name="/dev/ttyUSB0")
        self.Servo2.begin_communication()
        self.Servo2.set_operating_mode("extended position")

        self.Servo3 = Dynamixel(dxl_ids[2], descriptive_device_name="XM430 test motor", 
                series_name="xm", baudrate=bdrt, port_name="/dev/ttyUSB0")
        self.Servo3.begin_communication()
        self.Servo3.set_operating_mode("extended position")

        self.QJG = QJgeomerty(tendons_width)

        self.qualibrated = False
        self.servo1_zero = None
        self.servo2_zero = None
        self.servo3_zero = None

    def qualibrate(self):

        print("Qualibration in progress")

        self.servo1_zero = self.Servo1.read_position()
        self.servo2_zero = self.Servo2.read_position()
        self.servo3_zero = self.Servo3.read_position()

        tendon_a_streched = False
        tendon_b_streched = False
        tendon_c_streched = False

        while not ( tendon_a_streched and tendon_b_streched and tendon_c_streched):
            print("Motor average current : ", (self.Servo1.read_current() + self.Servo1.read_current() + self.Servo1.read_current())/3)
            
            if not tendon_a_streched:
                self.servo1_zero -= 3
                self.Servo1.write_position(self.servo1_zero)

                if np.abs(self.Servo1.read_current()) >= CURRENT_LIMIT:
                    tendon_a_streched = True

            if not tendon_b_streched:
                self.servo2_zero -= 3
                self.Servo2.write_position(self.servo2_zero)

                if np.abs(self.Servo2.read_current()) >= CURRENT_LIMIT:
                    tendon_b_streched = True

            if not tendon_c_streched:
                self.servo3_zero -= 3
                self.Servo3.write_position(self.servo3_zero)

                if np.abs(self.Servo3.read_current()) >= CURRENT_LIMIT:
                    tendon_c_streched = True

        self.qualibrated = True

        print("Qualibration successfull")

        return

    def mm2pos(self, d_mm):

        return d_mm * DYNAMIXELFULLROTATION / PULLEYDIAMETER / np.pi
    
    def rotate(self, theta, phi):
        if not self.qualibrated:
            print("Qualibrate first")
            return
        
        da, db, dc = self.QJG.set_angle(theta=theta, phi=phi)

        self.Servo1.write_position(self.servo1_zero + self.mm2pos(da))
        self.Servo2.write_position(self.servo2_zero + self.mm2pos(db))
        self.Servo3.write_position(self.servo3_zero + self.mm2pos(dc))

        return

    def disable(self):
        
        self.Servo1.end_communication()
        self.Servo2.end_communication()
        self.Servo3.end_communication()


    