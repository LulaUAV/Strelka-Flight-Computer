
from struct import *
import zlib
import os.path
from os import path
import time
import serial

class groundStation:
    def __init__(self):
        self.create_log_dir()
        
    def connect(self):
        # Connects to ground station receiver device
        ## **** Set this port to suit the serial port of the receiver device **** ##
        self.COMPort = "COM4"
        self.serialPort = serial.Serial(port = self.COMPort, baudrate=9600, bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE)
        time.sleep(2)
        # Send ACK packet
        self.serialPort.write(b'f')
        read_byte = self.serialPort.read()
        return  read_byte == b'\x66'
            
    def create_log_dir(self):
        if not os.path.exists("FlightData"):
            os.mkdir("FlightData")
        else:
            os.chdir("FlightData")
        self.log_dir = "/data"
        # print(directory)
        dirCount = 1
        while (path.exists(self.log_dir)):
            self.log_dir = "data"
            self.log_dir = self.log_dir+str(dirCount)
            dirCount+=1
        os.mkdir(self.log_dir)
        print("[groundStation] Writing data to: "+self.log_dir)
        self.logFile = open(self.log_dir+"/log.csv", "w")
        
    def write_log_dir(self, writeString):
        self.logFile.write(writeString)
    
    def calculate_crc32(self, received_bytes):
        return zlib.crc32(received_bytes[0:-4])

    def compare_crc32(self, received_bytes):
        rec_CRC = received_bytes[-4:-1]
        calc_CRC = self.calculate_crc32(received_bytes)
        return calc_CRC == rec_CRC
        
    def test_gimbal(self):
        self.serialPort.write(int(1))
        result = self.serialPort.read(6)
        if(self.compare_crc32(result)):
            # Valid packet
            return result[0] == (int(6) and result[1] == int(1))
        else: 
            # Invalid packet
            return -1
    
    def get_continuity(self):
        self.serialPort.write(int(2))
        result = self.serialPort.read(7)
        if(self.compare_crc32(result)):
            # Valid packet
            if(result[0] == int(2)):
                return [result[1], result[2]]
        else: 
            # Invalid packet
            return -1
    
    def get_battery_voltage(self):
        self.serialPort.write(int(3))
        result = self.serialPort.read(9)
        if(self.compare_crc32(result)):
            # Valid packet
            if(result[0] == (int(3))):
                [batt_volt] = unpack('f', result[1:4])
                return batt_volt
        else: 
            # Invalid 
            return -1

    # def get_baro_data(self):
    #     # Request last barometer data
        
    # def get_accel_data(self):
    #     # Request last accelerometer data 
        
    # def get_gyro_data(self):
    #     # Request last gyroscope data
        
    # def get_mag_data(self):
    #     # Request last magnetometer data
        
    # def get_gps_data(self):
    #     # Request last gps data
        
    # def get_state_vector(self):
    #     # Request system state vector from Kalman filter
        
        
        
    if __name__ == "__main__":
        print("This is a class, run groundStation.py for implementation")