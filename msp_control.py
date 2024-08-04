#!/usr/bin/env python3

import serial
import time
import glob

# MSP command codes
MSP_IDENT = 100
MSP_STATUS = 101
MSP_RAW_IMU = 102
MSP_SERVO = 103
MSP_MOTOR = 104
MSP_RC = 105
MSP_RAW_GPS = 106
MSP_COMP_GPS = 107
MSP_ATTITUDE = 108
MSP_ALTITUDE = 109
MSP_ANALOG = 110
MSP_RC_TUNING = 111
MSP_PID = 112
MSP_BOX = 113
MSP_MISC = 114
MSP_MOTOR_PINS = 115
MSP_BOXNAMES = 116
MSP_PIDNAMES = 117
MSP_WP = 118
MSP_BOXIDS = 119
MSP_SERVO_CONF = 120

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

def manual_control(speedybee_comm):
    print("Manual control mode. Enter commands:")
    print("1: MSP_IDENT, 2: MSP_STATUS, 3: MSP_RAW_IMU, 4: MSP_SERVO")
    print("5: MSP_MOTOR, 6: MSP_RC, 7: MSP_ATTITUDE, 8: MSP_ALTITUDE")
    print("q: Quit manual control")

    while True:
        command = input("Enter command: ").strip()
        if command == '1':
            speedybee_comm.send_msp_command(MSP_IDENT)
        elif command == '2':
            speedybee_comm.send_msp_command(MSP_STATUS)
        elif command == '3':
            speedybee_comm.send_msp_command(MSP_RAW_IMU)
        elif command == '4':
            speedybee_comm.send_msp_command(MSP_SERVO)
        elif command == '5':
            speedybee_comm.send_msp_command(MSP_MOTOR)
        elif command == '6':
            speedybee_comm.send_msp_command(MSP_RC)
        elif command == '7':
            speedybee_comm.send_msp_command(MSP_ATTITUDE)
        elif command == '8':
            speedybee_comm.send_msp_command(MSP_ALTITUDE)
        elif command == 'q':
            print("Exiting manual control")
            break
        else:
            print("Invalid command")

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
