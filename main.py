import sys
import os

submodule_path = os.path.abspath("dynamixel-controller")
sys.path.insert(0, submodule_path)

from time import sleep
from dynamixel_controller import Dynamixel
from QJ_kinematics import QuaternionJoint

import keyboard
import time

STEP_SIZE = 0.1
DELAY = 0.05

QuaterionJoint = QuaternionJoint()

print("=== Keyboard Velocity Control ===")
print("Use W/S to control Rotation x")
print("Use A/D to control Rotation y")

try:
    while True:
        if keyboard.is_pressed("q"):
            print("Exiting...")
            break

        # Motor 1: W/S
        if keyboard.is_pressed("w"):
            
            time.sleep(DELAY)
        elif keyboard.is_pressed("s"):

            time.sleep(DELAY)

        # Motor 2: A/D
        if keyboard.is_pressed("d"):

            time.sleep(DELAY)
        elif keyboard.is_pressed("a"):

            time.sleep(DELAY)

        # Motor 3: Z/X
        if keyboard.is_pressed("x"):

            time.sleep(DELAY)
        elif keyboard.is_pressed("z"):

            time.sleep(DELAY)

        time.sleep(0.01)

finally:
    QuaterionJoint.disable()
