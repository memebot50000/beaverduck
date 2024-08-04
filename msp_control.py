#!/usr/bin/env python3

import serial
import time
import glob
import threading

# MSP command codes
MSP_SET_RAW_RC = 200

# Helper function to create an MSP command
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
            response = self.ser.read(1024)
            if response:
                print(f"Received response from FC: {response.hex()}")
            else:
                print("No response from FC")
        except serial.SerialException as e:
            print(f"Error communicating with FC: {e}")
            self.ser = None

    def send_rc_data(self, channels):
        data = []
        for ch in channels:
            data += [ch & 0xFF, (ch >> 8) & 0xFF]
        self.send_msp_command(MSP_SET_RAW_RC, data)

def manual_control(speedybee_comm):
    print("Manual control mode. Use the following keys:")
    print("w/s: Increase/decrease throttle")
    print("a/d: Yaw left/right")
    print("i/k: Pitch forward/backward")
    print("j/l: Roll left/right")
    print("q: Quit manual control")

    channels = [1500, 1500, 1500, 1500]  # [Roll, Pitch, Throttle, Yaw]

    def update_channels():
        while True:
            command = input("Enter command: ").strip().lower()
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

            speedybee_comm.send_rc_data(channels)
            time.sleep(0.05)  # 20Hz update rate

    control_thread = threading.Thread(target=update_channels)
    control_thread.start()
    control_thread.join()

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
