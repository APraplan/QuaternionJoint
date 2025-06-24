import dynamixel_sdk as dsdk
import time

class DynamixelController:
    def __init__(self, dxl_ids=None, port="/dev/ttyUSB0", baudrate=57600, protocol_version=2.0):
        self.port = port
        self.baudrate = baudrate
        self.protocol_version = protocol_version
        self.dxl_ids = dxl_ids or [1]

        # Control table addresses for X-series (Protocol 2.0)
        self.ADDR_TORQUE_ENABLE = 64
        self.ADDR_GOAL_POSITION = 116
        self.ADDR_PRESENT_POSITION = 132

        # Initialize handlers
        self.portHandler = dsdk.PortHandler(self.port)
        self.packetHandler = dsdk.PacketHandler(self.protocol_version)

        # self.connect()

    def connect(self):
        if not self.portHandler.openPort():
            raise IOError("Failed to open port")
        if not self.portHandler.setBaudRate(self.baudrate):
            raise IOError("Failed to set baudrate")
        print("Connection successful")

    def enable_torque(self):
        for dxl_id in self.dxl_ids:
            self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, self.ADDR_TORQUE_ENABLE, 1)

    def disable_torque(self):
        for dxl_id in self.dxl_ids:
            self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, self.ADDR_TORQUE_ENABLE, 0)

    def get_position(self, dxl_id):
        pos, _, _ = self.packetHandler.read4ByteTxRx(self.portHandler, dxl_id, self.ADDR_PRESENT_POSITION)
        return pos

    def set_position(self, dxl_id, position):
        self.packetHandler.write4ByteTxRx(self.portHandler, dxl_id, self.ADDR_GOAL_POSITION, position)

    def increment_position(self, dxl_id, step):
        current = self.get_position(dxl_id)
        new_pos = current + step
        self.set_position(dxl_id, new_pos)
        return new_pos

    def close(self):
        self.disable_torque()
        self.portHandler.closePort()
        print("Connection closed")
