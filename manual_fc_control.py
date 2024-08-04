#!/usr/bin/env python3

import serial
import time
import glob

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

    def send_data(self, channels):
        if self.ser is None:
            print("No serial connection to FC")
            return

        formatted_data = bytearray([0x55, 0xAA])  # Start bytes
        for ch in channels:
            formatted_data += ch.to_bytes(2, byteorder='little')
        formatted_data += bytearray([0x00, 0x00])  # End bytes
        try:
            self.ser.write(formatted_data)
            print(f"Sent data to FC: {formatted_data.hex()}")
            
            # Try to read a response
            response = self.ser.read(100)
            if response:
                print(f"Received response from FC: {response.hex()}")
            else:
                print("No response from FC")
        except serial.SerialException as e:
            print(f"Error communicating with FC: {e}")
            self.ser = None

def manual_control(speedybee_comm):
    print("Manual control mode. Enter commands:")
    print("w: Increase throttle, s: Decrease throttle")
    print("a: Yaw left, d: Yaw right")
    print("i: Pitch forward, k: Pitch backward")
    print("j: Roll left, l: Roll right")
    print("q: Quit manual control")

    channels = [1500, 1500, 1500, 1500]  # [Roll, Pitch, Throttle, Yaw]
    
    while True:
        command = input("Enter command: ").lower()
        if command == 'w':
            channels[2] = min(2000, channels[2] + 10)  # Increase throttle
        elif command == 's':
            channels[2] = max(1000, channels[2] - 10)  # Decrease throttle
        elif command == 'a':
            channels[3] = max(1000, channels[3] - 10)  # Yaw left
        elif command == 'd':
            channels[3] = min(2000, channels[3] + 10)  # Yaw right
        elif command == 'i':
            channels[1] = min(2000, channels[1] + 10)  # Pitch forward
        elif command == 'k':
            channels[1] = max(1000, channels[1] - 10)  # Pitch backward
        elif command == 'j':
            channels[0] = max(1000, channels[0] - 10)  # Roll left
        elif command == 'l':
            channels[0] = min(2000, channels[0] + 10)  # Roll right
        elif command == 'q':
            print("Exiting manual control")
            break
        else:
            print("Invalid command")

        speedybee_comm.send_data(channels)
        time.sleep(0.05)  # 20Hz update rate

if __name__ == '__main__':
    speedybee_comm = SpeedybeeComm()

    print("\nStarting manual control mode...")
    try:
        manual_control(speedybee_comm)
    except KeyboardInterrupt:
        print("\nScript terminated by user")
    finally:
        print("Cleaning up...")
        if speedybee_comm.ser:
            speedybee_comm.ser.close()
