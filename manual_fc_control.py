#!/usr/bin/env python3

import serial
import time
import glob
import keyboard

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
            print(f"Sent data to FC: {channels}")
        except serial.SerialException as e:
            print(f"Error sending data to FC: {e}")
            self.ser = None

def manual_control(speedybee_comm):
    print("Manual control mode. Use the following keys:")
    print("W/S: Throttle up/down")
    print("A/D: Yaw left/right")
    print("I/K: Pitch forward/backward")
    print("J/L: Roll left/right")
    print("Q: Quit manual control")

    channels = [1500, 1500, 1500, 1500]  # [Roll, Pitch, Throttle, Yaw]
    
    while True:
        if keyboard.is_pressed('w'):
            channels[2] = min(2000, channels[2] + 10)  # Increase throttle
        elif keyboard.is_pressed('s'):
            channels[2] = max(1000, channels[2] - 10)  # Decrease throttle
        elif keyboard.is_pressed('a'):
            channels[3] = max(1000, channels[3] - 10)  # Yaw left
        elif keyboard.is_pressed('d'):
            channels[3] = min(2000, channels[3] + 10)  # Yaw right
        elif keyboard.is_pressed('i'):
            channels[1] = min(2000, channels[1] + 10)  # Pitch forward
        elif keyboard.is_pressed('k'):
            channels[1] = max(1000, channels[1] - 10)  # Pitch backward
        elif keyboard.is_pressed('j'):
            channels[0] = max(1000, channels[0] - 10)  # Roll left
        elif keyboard.is_pressed('l'):
            channels[0] = min(2000, channels[0] + 10)  # Roll right
        elif keyboard.is_pressed('q'):
            print("Exiting manual control")
            break

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
