#!/usr/bin/env python3

import serial
import time
import glob
import threading
import os

class SpektrumReceiver:
    def __init__(self, port='/dev/ttyAMA0'):
        self.port = port
        self.baudrates = [115200, 115000, 100000, 125000]  # Common Spektrum baud rates
        self.ser = None
        self.channels = [0, 0, 0, 0]

    def open_serial(self):
        for baudrate in self.baudrates:
            try:
                self.ser = serial.Serial(self.port, baudrate, timeout=1)
                print(f"Successfully opened serial port {self.port} at {baudrate} baud")
                return True
            except serial.SerialException as e:
                print(f"Failed to open serial port at {baudrate} baud: {e}")
        return False

    def read_spektrum(self):
        while True:
            if self.ser is None or not self.ser.is_open:
                print("Serial port not open. Attempting to open...")
                if not self.open_serial():
                    print("Failed to open serial port. Retrying in 5 seconds...")
                    time.sleep(5)
                    continue

            try:
                if self.ser.in_waiting > 0:
                    data = self.ser.read(16)  # Always try to read 16 bytes
                    print(f"Raw data ({len(data)} bytes): {data.hex()}")
                    if len(data) == 16 and data[0] == 0x0F and data[1] == 0xA2:
                        for i in range(4):  # Only read 4 channels
                            ch = (data[2*i+3] << 8) | data[2*i+2]
                            self.channels[i] = ch & 0x07FF
                        print(f"Channel values: {self.channels}")
                    else:
                        print(f"Received data, but not in expected format. First two bytes: {data[:2].hex()}")
                else:
                    print("Waiting for data...")
            except serial.SerialException as
