import sys
import os

submodule_path = os.path.abspath("dynamixel-controller")
sys.path.insert(0, submodule_path)

from time import sleep
from dynamixel_controller_fast import DynamixelController, BaseModel
from QJ_geomerty import QJgeomerty
from Encoder import AMT23_Encoder

import keyboard
import time
import numpy as np

PULLEYDIAMETER = 20
DYNAMIXELFULLROTATION = 4096

TARGET_VID = 6790
TARGET_PID = 29987

CURRENT_MIN = 50
CURRENT_MAX = 500

KP = 0.4
KI = 0.0
KD = 0.0


class QuaternionJointK:
    def __init__(self, dxl_ids = [2, 3, 1], bdrt = 57600, tendons_width = 75):

        port = "/dev/ttyUSB1"

        motor_list = []
        for i in range(len(dxl_ids)):
            motor_list.append(BaseModel(dxl_ids[i]))

        self.controller = DynamixelController(port, motor_list, baudrate=bdrt, latency_time=10)

        self.controller.activate_controller()
        self.controller.set_operating_mode([0, 0, 0])

        # self.Servo1 = Dynamixel(dxl_ids[0], descriptive_device_name="XM430 test motor", 
        #         series_name="xm", baudrate=bdrt, port_name=port)
        # self.Servo1.begin_communication()
        # self.Servo1.set_operating_mode("current")

        # self.Servo2 = Dynamixel(dxl_ids[1], descriptive_device_name="XM430 test motor", 
        #         series_name="xm", baudrate=bdrt, port_name=port)
        # self.Servo2.begin_communication()
        # self.Servo2.set_operating_mode("current")

        # self.Servo3 = Dynamixel(dxl_ids[2], descriptive_device_name="XM430 test motor", 
        #         series_name="xm", baudrate=bdrt, port_name=port)
        # self.Servo3.begin_communication()
        # self.Servo3.set_operating_mode("current")

        self.encoders = AMT23_Encoder()
        self.encoders.connect(TARGET_VID, TARGET_PID)

        self.QJG = QJgeomerty(tendons_width)

        self.qualibrated = False
        self.servo1_zero = None
        self.servo2_zero = None
        self.servo3_zero = None

        self.current_targets = [0, 0, 0]

        self.pid1 = PID(kp=KP, ki=KI, kd=KD)
        self.pid2 = PID(kp=KP, ki=KI, kd=KD)
        self.pid3 = PID(kp=KP, ki=KI, kd=KD)

    def qualibrate(self):

        print("Qualibration in progress")

        # self.servo1_zero = self.Servo1.read_position()
        # self.servo2_zero = self.Servo2.read_position()
        # self.servo3_zero = self.Servo3.read_position()

        # tendon_a_streched = False
        # tendon_b_streched = False
        # tendon_c_streched = False

        # while not ( tendon_a_streched and tendon_b_streched and tendon_c_streched):
        #     print("Motor average current : ", (self.Servo1.read_current() + self.Servo1.read_current() + self.Servo1.read_current())/3)
            
        #     if not tendon_a_streched:
        #         self.servo1_zero -= 5
        #         self.Servo1.write_position(self.servo1_zero)

        #         if np.abs(self.Servo1.read_current()) >= CURRENT_LIMIT:
        #             tendon_a_streched = True

        #     if not tendon_b_streched:
        #         self.servo2_zero -= 5
        #         self.Servo2.write_position(self.servo2_zero)

        #         if np.abs(self.Servo2.read_current()) >= CURRENT_LIMIT:
        #             tendon_b_streched = True

        #     if not tendon_c_streched:
        #         self.servo3_zero -= 5
        #         self.Servo3.write_position(self.servo3_zero)

        #         if np.abs(self.Servo3.read_current()) >= CURRENT_LIMIT:
        #             tendon_c_streched = True

        self.controller.torque_on()
        self.controller.set_goal_current_mA([-CURRENT_MIN, -CURRENT_MIN, -CURRENT_MIN])
        print(self.encoders.read_position())

        # self.Servo1.write_current(-CURRENT_MIN)
        # self.Servo2.write_current(-CURRENT_MIN)
        # self.Servo3.write_current(-CURRENT_MIN)    
                    
        # self.encoders.calibrate() 

        self.qualibrated = True

        print("Qualibration successfull")

        return

    def mm2pos(self, d_mm):

        return d_mm * DYNAMIXELFULLROTATION / PULLEYDIAMETER / np.pi
    
    def rotate(self, target_theta, target_phi, measured_theta, measured_phi):
        if not self.qualibrated:
            print("Qualibrate first")
            return
        
        ta, tb, tc = self.QJG.set_angle(theta=target_theta, phi=target_phi-np.pi/2)
        ma, mb, mc = self.QJG.set_angle(theta=measured_theta, phi=measured_phi-np.pi/2)

        self.current_targets[0] = self.pid1.compute(setpoint=self.mm2pos(ta), measured_value=self.mm2pos(ma), dt=1)
        self.current_targets[1] = self.pid2.compute(setpoint=self.mm2pos(tb), measured_value=self.mm2pos(mb), dt=1)
        self.current_targets[2] = self.pid3.compute(setpoint=self.mm2pos(tc), measured_value=self.mm2pos(mc), dt=1)


        # print("Target currents : ", self.current_targets)

        # self.Servo1.write_current(self.current_targets[0])
        # self.Servo2.write_current(self.current_targets[1])
        # self.Servo3.write_current(self.current_targets[2])  

        self.controller.set_goal_current_mA(self.current_targets)

        return
    
    def read_angles(self):
        angle1, angle2 = self.encoders.read_angle()

        theta, phi = self.QJG.compute_theta_phi(angle1=2*angle1, angle1_pos=np.deg2rad(60), angle2=2*angle2, angle2_pos=np.deg2rad(180))

        return theta, phi

    def disable(self):

        self.controller.torque_off()
        
        # self.Servo1.disable_torque()
        # self.Servo2.disable_torque()
        # self.Servo3.disable_torque()

        # self.Servo1.end_communication()
        # self.Servo2.end_communication()
        # self.Servo3.end_communication()


class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.prev_error = 0
        self.integral = 0
        self.anti_windup_limit = 2500

    def compute(self, setpoint, measured_value, dt):
        error = setpoint - measured_value
        self.integral += error * dt

        # print(self.integral)
        
        if self.integral > self.anti_windup_limit:
            self.integral = self.anti_windup_limit
        elif self.integral < -self.anti_windup_limit:
            self.integral = -self.anti_windup_limit

        derivative = (error - self.prev_error) / dt if dt > 0 else 0

        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)

        self.prev_error = error

        if output > -CURRENT_MIN:
            output = -CURRENT_MIN
        elif output < -CURRENT_MAX:
            output = -CURRENT_MAX

        return output

    