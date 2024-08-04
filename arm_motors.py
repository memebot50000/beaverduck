#!/usr/bin/env python3

import serial
import time
import glob

# MSP command codes
MSP_ARM = 216
MSP_DISARM = 217

def create_msp_command(cmd, data=[]):
    length = len(data)
    checksum = length ^ cmd
    for byte in data:
        checksum ^= byte
    return bytearray([ord('$'), ord('M'), ord('<'), length, cmd] + data + [checksum])

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
            except serial.SerialException as e:
                print(f"Failed to connect on {port}: {e}")
        print("Could not connect to Speedybee FC")

    def send_msp_command(self, cmd, data=[]):
        if self.ser is None:
            print("No serial connection to FC")
            return

        command = create_msp_command(cmd, data)
        try:
            self.ser.write(command)
            print(f"Sent MSP command to FC: {command.hex()}")

            # Try to read a response
            response = self.ser.read(6)  # We expect a 6-byte response
            if response:
                print(f"Received response from FC: {response.hex()}")
            else:
                print("No response from FC")
        except serial.SerialException as e:
            print(f"Error communicating with FC: {e}")
            self.ser = None

    def arm_motors(self):
        print("Attempting to arm motors...")
        self.send_msp_command(MSP_ARM)
        time.sleep(1)  # Wait for a second after arming

    def disarm_motors(self):
        print("Disarming motors...")
        self.send_msp_command(MSP_DISARM)

if __name__ == '__main__':
    speedybee_comm = SpeedybeeComm()

    try:
        while True:
            command = input("Enter 'arm' to arm motors, 'disarm' to disarm, or 'quit' to exit: ").strip().lower()
            if command == 'arm':
                speedybee_comm.arm_motors()
            elif command == 'disarm':
                speedybee_comm.disarm_motors()
            elif command == 'quit':
                break
            else:
                print("Invalid command")
    except KeyboardInterrupt:
        print("\nScript terminated by user")
    finally:
        print("Cleaning up...")
        if speedybee_comm.ser:
            speedybee_comm.ser.close()
