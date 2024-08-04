#!/usr/bin/env python3

import serial
import time
import glob

class SpeedybeeComm:
    def __init__(self):
        self.ser = None
        self.find_and_connect_port()

    def find_and_connect_port(self):
        # List all available USB serial ports
        ports = glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*')
        
        if not ports:
            print("No USB serial ports found. Is the Speedybee FC connected?")
            return

        for port in ports:
            try:
                self.ser = serial.Serial(port, 115200, timeout=1)
                print(f"Connected to Speedybee FC on {port}")
                return
            except serial.SerialException:
                print(f"Failed to connect on {port}")

        if self.ser is None:
            print("Could not connect to Speedybee FC on any port")

    def send_data(self, channels):
        if self.ser is None:
            print("Not connected to Speedybee FC")
            return

        formatted_data = bytearray([0x55, 0xAA])  # Start bytes
        for ch in channels:
            formatted_data += ch.to_bytes(2, byteorder='little')
        formatted_data += bytearray([0x00, 0x00])  # End bytes
        self.ser.write(formatted_data)

    def run(self):
        try:
            while True:
                if self.ser is None:
                    print("Attempting to reconnect...")
                    self.find_and_connect_port()
                    time.sleep(5)  # Wait 5 seconds before trying again
                    continue

                # For testing, send dummy data
                dummy_channels = [1000, 1500, 2000, 1750]
                self.send_data(dummy_channels)
                print(f"Sent data: {dummy_channels}")
                time.sleep(0.02)  # 50Hz update rate
        except KeyboardInterrupt:
            pass
        finally:
            if self.ser:
                self.ser.close()

if __name__ == '__main__':
    comm = SpeedybeeComm()
    comm.run()
