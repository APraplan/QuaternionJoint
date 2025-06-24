from QJ_dynamixel import DynamixelController
import keyboard
import keyboard
import time

STEP_SIZE = 20
DELAY = 0.05

DXL_IDS = [1, 2, 3]

dxl = DynamixelController(DXL_IDS)

dxl.connect()
dxl.enable_torque()

print("=== Keyboard Velocity Control ===")
print("Use W/S to control motor 1")
print("Use A/D to control motor 2")
print("Press Q to quit")

try:
    while True:
        if keyboard.is_pressed("q"):
            print("Exiting...")
            break

        # Motor 1: W/S
        if keyboard.is_pressed("w"):
            new_pos = dxl.increment_position(1, STEP_SIZE)
            print(f"[ID:1] + -> {new_pos}")
            time.sleep(DELAY)
        elif keyboard.is_pressed("s"):
            new_pos = dxl.increment_position(1, -STEP_SIZE)
            print(f"[ID:1] - -> {new_pos}")
            time.sleep(DELAY)

        # Motor 2: A/D
        if keyboard.is_pressed("d"):
            new_pos = dxl.increment_position(2, STEP_SIZE)
            print(f"[ID:2] + -> {new_pos}")
            time.sleep(DELAY)
        elif keyboard.is_pressed("a"):
            new_pos = dxl.increment_position(2, -STEP_SIZE)
            print(f"[ID:2] - -> {new_pos}")
            time.sleep(DELAY)

        # Motor 3: Z/X
        if keyboard.is_pressed("x"):
            new_pos = dxl.increment_position(3, STEP_SIZE)
            print(f"[ID:3] + -> {new_pos}")
            time.sleep(DELAY)
        elif keyboard.is_pressed("z"):
            new_pos = dxl.increment_position(3, -STEP_SIZE)
            print(f"[ID:3] - -> {new_pos}")
            time.sleep(DELAY)

        time.sleep(0.01)

finally:
    dxl.close()