import sys
import os

from time import sleep
from QJ_kinematics import QuaternionJointK

import keyboard
import time
import numpy as np

STEP_SIZE_THETA = 0.015
STEP_SIZE_PHI = 0.02
DELAY = 0.05

theta = 0
phi = 0

QuaternionJoint = QuaternionJointK()
# QuaternionJoint.qualibrate()

print("=== Keyboard Velocity Control ===")
print("Use W/S to control Rotation x")
print("Use A/D to control Rotation y")
print("Use C to calibrate")

try:
    while not keyboard.is_pressed("c"):
        pass

    QuaternionJoint.qualibrate()
    

    while True:
        if keyboard.is_pressed("q"):
            print("Exiting...")
            break

        if keyboard.is_pressed("c"):
            QuaternionJoint.qualibrate()

        if keyboard.is_pressed("k"):
            print("Demo started")

            ROTATION_PERIOD_YAW = 600
            OSC_PERIOD = 150
            TWO_PI = 2*np.pi
            PITCH_OSC_MID = 0.96
            PITCH_OSC_AMP = 0.34

            osc_omega = TWO_PI / OSC_PERIOD
            rotation_speed = TWO_PI / ROTATION_PERIOD_YAW
            
            start_time = time.time()

            

            while True:

                current_time = time.time() - start_time

                phi = (rotation_speed * current_time) % TWO_PI
    
                theta = PITCH_OSC_MID + PITCH_OSC_AMP * np.sin(osc_omega * current_time)

                QuaternionJoint.rotate(theta=theta, phi=phi)

                time.sleep(0.001)
    

        # Theta: W/S
        if keyboard.is_pressed("w"):
            theta -= STEP_SIZE_THETA
        elif keyboard.is_pressed("s"):
            theta += STEP_SIZE_THETA

        # Phi: A/Dq
        if keyboard.is_pressed("d"):
            phi -= STEP_SIZE_PHI
        elif keyboard.is_pressed("a"):
            phi += STEP_SIZE_PHI

        QuaternionJoint.rotate(theta=theta, phi=phi)

        time.sleep(0.01)

finally:
    QuaternionJoint.disable()
