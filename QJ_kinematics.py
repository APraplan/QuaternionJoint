import sys
import os

submodule_path = os.path.abspath("dynamixel-controller")
sys.path.insert(0, submodule_path)

from time import sleep
from dynamixel_controller import Dynamixel

import keyboard
import time


class QuaternionJoint:
    def __init__(self, dxl_ids = [1, 2, 3], bdrt = 57600):
        

        self.Servo1 = Dynamixel(dxl_ids[0], descriptive_device_name="XM430 test motor", 
                series_name="xm", baudrate=bdrt, port_name="/dev/ttyUSB0")
        self.Servo1.begin_communication()
        self.Servo1.set_operating_mode("position")

        self.Servo2 = Dynamixel(dxl_ids[1], descriptive_device_name="XM430 test motor", 
                series_name="xm", baudrate=bdrt, port_name="/dev/ttyUSB0")
        self.Servo2.begin_communication()
        self.Servo2.set_operating_mode("position")

        self.Servo3 = Dynamixel(dxl_ids[2], descriptive_device_name="XM430 test motor", 
                series_name="xm", baudrate=bdrt, port_name="/dev/ttyUSB0")
        self.Servo3.begin_communication()
        self.Servo3.set_operating_mode("position")

    def disable(self):
        
        self.Servo1.end_communication()
        self.Servo2.end_communication()
        self.Servo3.end_communication()


    