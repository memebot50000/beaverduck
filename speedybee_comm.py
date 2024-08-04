#!/usr/bin/env python3

import serial
import time

class SpeedybeeComm:
    def __init__(self):
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

    def send_data(self, channels):
        formatted_data = bytearray([0x55, 0xAA])  # Start bytes
        for ch in channels:
            formatted_data += ch.to_bytes(2, byteorder='little')
        formatted_data += bytearray([0x00, 0x00])  # End bytes
        self.ser.write(formatted_data)

    def run(self):
        try:
            while True:
                # For testing, send dummy data
                dummy_channels = [1000, 1500, 2000, 1750]
                self.send_data(dummy_channels)
                print(f"Sent data: {dummy_channels}")
                time.sleep(0.02)  # 50Hz update rate
        except KeyboardInterrupt:
            pass
        finally:
            self.ser.close()

if __name__ == '__main__':
    comm = SpeedybeeComm()
    comm.run()
