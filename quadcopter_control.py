#!/usr/bin/env python3

import serial
import time
import glob
import threading
import os
import subprocess

class SpektrumReceiver:
    def __init__(self, port='/dev/ttyAMA0'):
        self.port = port
        self.baudrate = 115200  # Spektrum AR410 typically uses 115200 baud
        self.ser = None
        self.channels = [1500, 1500, 1500, 1500]  # Default center values
        self.lock = threading.Lock()

    def open_serial(self):
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1
            )
            print(f"Successfully opened serial port {self.port} at {self.baudrate} baud")
            return True
        except serial.SerialException as e:
            print(f"Failed to open serial port: {e}")
            return False

    def read_spektrum(self):
        if not self.open_serial():
            print("Failed to open Spektrum receiver serial port")
            return

        while True:
            try:
                if self.ser.in_waiting >= 16:
                    data = self.ser.read(16)
                    if data[0] == 0x0F and data[1] == 0xA2:
                        with self.lock:
                            for i in range(4):
                                ch = (data[2*i+3] << 8) | data[2*i+2]
                                self.channels[i] = ch & 0x07FF
                        print(f"Channel values: {self.channels}")
                    else:
                        print(f"Received unexpected data: {data.hex()}")
            except serial.SerialException as e:
                print(f"Serial error: {e}")
                time.sleep(1)
                if not self.open_serial():
                    print("Failed to reopen Spektrum receiver serial port")
                    time.sleep(5)

            time.sleep(0.01)  # 100Hz update rate

class SpeedybeeComm:
    def __init__(self):
        self.ser = None
        self.find_and_connect_port()

    def find_and_connect_port(self):
        ports = glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*')
        for port in ports:
            try:
                self.ser = serial.Serial(port, 115200, timeout=1)
                print(f"Connected to Speedybee FC on {port}")
                return
            except serial.SerialException:
                pass
        print("Could not connect to Speedybee FC")

    def send_data(self, channels):
        if self.ser is None:
            return

        formatted_data = bytearray([0x55, 0xAA])  # Start bytes
        for ch in channels:
            formatted_data += ch.to_bytes(2, byteorder='little')
        formatted_data += bytearray([0x00, 0x00])  # End bytes
        try:
            self.ser.write(formatted_data)
        except serial.SerialException as e:
            print(f"Error sending data to FC: {e}")
            self.ser = None

    def run(self, spektrum_receiver):
        while True:
            if self.ser is None:
                self.find_and_connect_port()
                if self.ser is None:
                    time.sleep(5)
                    continue

            with spektrum_receiver.lock:
                channels = spektrum_receiver.channels.copy()
            self.send_data(channels)
            time.sleep(0.02)  # 50Hz update rate

def setup_uart():
    # Disable console on UART
    os.system("sudo systemctl stop serial-getty@ttyAMA0.service")
    os.system("sudo systemctl disable serial-getty@ttyAMA0.service")
    
    # Enable UART in config.txt if not already enabled
    with open('/boot/config.txt', 'r') as f:
        config = f.read()
    if 'enable_uart=1' not in config:
        with open('/boot/config.txt', 'a') as f:
            f.write('\nenable_uart=1\n')
        print("Added enable_uart=1 to /boot/config.txt. Please reboot for changes to take effect.")
    
    # Remove console from cmdline.txt if present
    with open('/boot/cmdline.txt', 'r') as f:
        cmdline = f.read()
    if 'console=serial0,115200' in cmdline:
        cmdline = cmdline.replace('console=serial0,115200', '')
        with open('/boot/cmdline.txt', 'w') as f:
            f.write(cmdline)
        print("Removed console=serial0,115200 from /boot/cmdline.txt. Please reboot for changes to take effect.")

if __name__ == '__main__':
    setup_uart()
    
    spektrum_receiver = SpektrumReceiver()
    speedybee_comm = SpeedybeeComm()

    # Start Spektrum receiver thread
    receiver_thread = threading.Thread(target=spektrum_receiver.read_spektrum)
    receiver_thread.start()

    # Start Speedybee communication in the main thread
    try:
        speedybee_comm.run(spektrum_receiver)
    except KeyboardInterrupt:
        print("Script terminated by user")
    finally:
        print("Cleaning up...")
        if spektrum_receiver.ser:
            spektrum_receiver.ser.close()
        if speedybee_comm.ser:
            speedybee_comm.ser.close()
