#!/usr/bin/env python3

import serial
import time
import glob
import threading
import os
import subprocess
import RPi.GPIO as GPIO

def check_uart_config():
    try:
        config = subprocess.check_output(['vcgencmd', 'get_config', 'uart0']).decode('utf-8')
        print("UART Configuration:")
        print(config)
    except Exception as e:
        print(f"Error checking UART configuration: {e}")

def check_gpio_config():
    try:
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(15, GPIO.IN)  # Set up GPIO 15 as input
        state = GPIO.input(15)
        print(f"GPIO 15 state: {'HIGH' if state else 'LOW'}")
    except Exception as e:
        print(f"Error checking GPIO configuration: {e}")

class SpektrumReceiver:
    def __init__(self, port='/dev/ttyAMA0'):
        self.port = port
        self.baudrates = [115200, 115000, 100000, 125000, 9600]
        self.ser = None
        self.channels = [0, 0, 0, 0]

    def open_serial(self):
        for baudrate in self.baudrates:
            try:
                self.ser = serial.Serial(
                    port=self.port,
                    baudrate=baudrate,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=1
                )
                print(f"Successfully opened serial port {self.port} at {baudrate} baud")
                return True
            except serial.SerialException as e:
                print(f"Failed to open serial port at {baudrate} baud: {e}")
        return False

    def read_spektrum(self):
        zero_count = 0
        while True:
            if self.ser is None or not self.ser.is_open:
                print("Serial port not open. Attempting to open...")
                if not self.open_serial():
                    print("Failed to open serial port. Retrying in 5 seconds...")
                    time.sleep(5)
                    continue

            try:
                if self.ser.in_waiting > 0:
                    data = self.ser.read(self.ser.in_waiting)
                    print(f"Raw data ({len(data)} bytes): {data.hex()}")
                    print(f"ASCII representation: {data}")
                    if all(b == 0 for b in data):
                        zero_count += 1
                        if zero_count > 10:
                            print("Received too many zero packets. Resetting serial connection...")
                            self.ser.close()
                            self.ser = None
                            zero_count = 0
                            continue
                    else:
                        zero_count = 0

                    if len(data) >= 16 and data[0] == 0x0F and data[1] == 0xA2:
                        for i in range(4):
                            ch = (data[2*i+3] << 8) | data[2*i+2]
                            self.channels[i] = ch & 0x07FF
                        print(f"Channel values: {self.channels}")
                    else:
                        print(f"Received data, but not in expected format. All bytes: {[hex(b) for b in data]}")
                else:
                    print("Waiting for data...")
            except serial.SerialException as e:
                print(f"Serial error: {e}")
                self.ser = None

            time.sleep(0.1)

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
            print(f"Received response from FC: {response.hex()}")
        else:
            print("No response from FC")

    def run(self, spektrum_receiver):
        try:
            while True:
                if self.ser is None:
                    print("Attempting to reconnect to Speedybee FC...")
                    self.find_and_connect_port()
                    time.sleep(5)  # Wait 5 seconds before trying again
                    continue

                self.send_data(spektrum_receiver.channels)
                print(f"Sent data to FC: {spektrum_receiver.channels}")
                time.sleep(0.1)  # Reduced rate for debugging
        except KeyboardInterrupt:
            pass
        finally:
            if self.ser:
                self.ser.close()

if __name__ == '__main__':
    print("System Information:")
    print(f"Python version: {os.sys.version}")
    print(f"Operating system: {os.name}")
    print(f"Platform: {os.sys.platform}")
    
    print("\nAdditional System Information:")
    print("UART configuration in /boot/config.txt:")
    os.system("grep uart /boot/config.txt")
    
    print("\nGPIO Information:")
    check_gpio_config()
    
    print("\nUART Configuration:")
    check_uart_config()
    
    if os.path.exists('/dev/ttyAMA0'):
        print("\nttyAMA0 exists")
    else:
        print("\nttyAMA0 does not exist")
    
    tty_devices = glob.glob('/dev/tty*')
    print("\nAvailable tty devices:")
    for device in tty_devices:
        print(device)

    spektrum_receiver = SpektrumReceiver()
    speedybee_comm = SpeedybeeComm()

    print("\nStarting Spektrum receiver and Speedybee communication...")
    receiver_thread = threading.Thread(target=spektrum_receiver.read_spektrum)
    receiver_thread.start()

    try:
        speedybee_comm.run(spektrum_receiver)
    except KeyboardInterrupt:
        print("\nScript terminated by user.")
    finally:
        print("Cleaning up...")
        GPIO.cleanup()
