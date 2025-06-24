import sys
import os

submodule_path = os.path.abspath("dynamixel-controller")
sys.path.insert(0, submodule_path)

from time import sleep
from dynamixel_controller import Dynamixel
from QJ_kinematics import QuaternionJointK

import keyboard
import time

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

        # Theta: W/S
        if keyboard.is_pressed("w"):
            theta -= STEP_SIZE_THETA
        elif keyboard.is_pressed("s"):
            theta += STEP_SIZE_THETA

        # Phi: A/D
        if keyboard.is_pressed("d"):
            phi -= STEP_SIZE_PHI
        elif keyboard.is_pressed("a"):
            phi += STEP_SIZE_PHI

        QuaternionJoint.rotate(theta=theta, phi=phi)

        time.sleep(0.01)

finally:
    QuaternionJoint.disable()
