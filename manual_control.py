#!/usr/bin/env python3

import serial
import time
import glob
import threading

class SpektrumReceiver:
    def __init__(self, port='/dev/ttyAMA0', baudrate=115200):
        try:
            self.ser = serial.Serial(port, baudrate, timeout=1)
            print(f"Successfully opened serial port {port}")
        except serial.SerialException as e:
            print(f"Failed to open serial port: {e}")
            self.ser = None
        self.channels = [0, 0, 0, 0]

    def read_spektrum(self):
        while True:
            if self.ser is None:
                print("Serial port not open. Retrying in 5 seconds...")
                time.sleep(5)
                try:
                    self.ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=1)
                    print("Successfully opened serial port /dev/ttyAMA0")
                except serial.SerialException as e:
                    print(f"Failed to open serial port: {e}")
                continue

            try:
                if self.ser.in_waiting >= 16:
                    data = self.ser.read(16)
                    print(f"Raw data: {data.hex()}")
                    if data[0] == 0x0F and data[1] == 0xA2:
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
    spektrum_receiver = SpektrumReceiver()
    speedybee_comm = SpeedybeeComm()

    # Run SpektrumReceiver in a separate thread
    receiver_thread = threading.Thread(target=spektrum_receiver.read_spektrum)
    receiver_thread.start()

    # Run SpeedybeeComm in the main thread
    speedybee_comm.run(spektrum_receiver)
