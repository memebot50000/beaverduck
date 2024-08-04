#!/usr/bin/env python3

import serial
import time
import glob
import threading
import os

class SpektrumReceiver:
    def __init__(self, port='/dev/ttyAMA0', baudrate=115000):  # Try 115000 first
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.channels = [0, 0, 0, 0]

    def open_serial(self):
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            print(f"Successfully opened serial port {self.port} at {self.baudrate} baud")
            return True
        except serial.SerialException as e:
            print(f"Failed to open serial port: {e}")
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
                    print(f"Bytes available: {self.ser.in_waiting}")
                    data = self.ser.read(self.ser.in_waiting)
                    print(f"Raw data: {data.hex()}")
                    if len(data) >= 16 and data[0] == 0x0F and data[1] == 0xA2:
                        for i in range(4):  # Only read 4 channels
                            ch = (data[2*i+3] << 8) | data[2*i+2]
                            self.channels[i] = ch & 0x07FF
                        print(f"Channel values: {self.channels}")
                    else:
                        print("Received data, but not in expected format")
                else:
                    print("Waiting for data...")
            except serial.SerialException as e:
                print(f"Serial error: {e}")
                self.ser = None

            time.sleep(0.1)  # Reduced rate for debugging

class SpeedybeeComm:
    def __init__(self):
        self.ser = None
        self.find_and_connect_port()

    def find_and_connect_port(self):
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
        
        # Try to read any response from the FC
        response = self.ser.read(100)  # Read up to 100 bytes
        if response:
            print(f"Received response: {response}")
        else:
            print("No response from FC")

    def run(self, spektrum_receiver):
        try:
            while True:
                if self.ser is None:
                    print("Attempting to reconnect...")
                    self.find_and_connect_port()
                    time.sleep(5)  # Wait 5 seconds before trying again
                    continue

                self.send_data(spektrum_receiver.channels)
                print(f"Sent data: {spektrum_receiver.channels}")
                time.sleep(0.1)  # Reduced rate for debugging
        except KeyboardInterrupt:
            pass
        finally:
            if self.ser:
                self.ser.close()

if __name__ == '__main__':
    # Print system information
    print("System Information:")
    print(f"Python version: {os.sys.version}")
    print(f"Operating system: {os.name}")
    print(f"Platform: {os.sys.platform}")
    
    # Check if ttyAMA0 exists
    if os.path.exists('/dev/ttyAMA0'):
        print("ttyAMA0 exists")
    else:
        print("ttyAMA0 does not exist")
    
    # Try to open ttyAMA0
    try:
        with serial.Serial('/dev/ttyAMA0', 115000, timeout=1) as ser:
            print("Successfully opened ttyAMA0")
    except Exception as e:
        print(f"Failed to open ttyAMA0: {e}")

    spektrum_receiver = SpektrumReceiver()
    speedybee_comm = SpeedybeeComm()

    # Run SpektrumReceiver in a separate thread
    receiver_thread = threading.Thread(target=spektrum_receiver.read_spektrum)
    receiver_thread.start()

    # Run SpeedybeeComm in the main thread
    speedybee_comm.run(spektrum_receiver)
