import serial
import serial.tools.list_ports
import struct
import numpy as np

# Replace these with your device's actual VID and PID
TARGET_VID = 6790
TARGET_PID = 29987

ECONDERFULLROTATION = 4096

class AMT23_Encoder():
    def __init__(self):
        self.ser = None

        self.calibrated = True
        
        self.encoder1_zero = 2360
        self.encoder2_zero = 2370

    def connect(self, vid, pid):
        port = self.find_serial_port(vid=vid, pid=pid)
        
        if port:
            self.ser = serial.Serial(port, 115200, timeout=1)

    def find_serial_port(sefl, vid, pid):
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if port.vid is not None and port.pid is not None:
                print(f"Available device: {port.device} (VID: {vid:04X}, PID: {pid:04X})")
                if port.vid == vid and port.pid == pid:
                    print(f"Found device: {port.device} (VID: {vid:04X}, PID: {pid:04X})")
                    return port.device
        print("Device not found.")
        return None
    
    def read_position(self):

        self.ser.reset_input_buffer()

        while True:

            # Wait for start byte
            byte = self.ser.read(1)
            if not byte or byte[0] != 0xAA:
                continue

            # Read two uint16_t values (4 bytes)
            data = self.ser.read(4)
            if len(data) != 4:
                continue

            # Read end byte
            end = self.ser.read(1)
            if not end or end[0] != 0x55:
                continue

            # Unpack and print the values in decimal
            value1, value2 = struct.unpack('<HH', data)
            return value1, value2

    def calibrate(self):
        self.read_position()
        self.encoder1_zero, self.encoder2_zero = self.read_position()
        self.calibrated = True
        print("Encoder calibrated")

        return
    
    def read_angle(self):
        if not self.calibrated:
            print("Please calibrate encoders first")
            return None
        else:
            pos1, pos2 = self.read_position()
            angle1 = (pos1 - self.encoder1_zero)/ECONDERFULLROTATION*2*np.pi
            angle2 = (pos2 - self.encoder2_zero)/ECONDERFULLROTATION*2*np.pi

            return angle1, angle2
    
    def disconect(self):
        self.ser.close()

        return
    

if __name__ == "__main__":
    encoder = AMT23_Encoder()
    encoder.connect(vid=TARGET_VID, pid=TARGET_PID)

    if encoder.ser is None:
        print("Could not connect to the encoder")
    else:
        print("Connected to the encoder")

        encoder.calibrate()

        while True:
            angle1, angle2 = encoder.read_angle()
            print("angle1 : ", np.rad2deg(angle1), " angle2 : ", np.rad2deg(angle2))
        
        encoder.disconect()