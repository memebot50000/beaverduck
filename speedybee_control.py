import serial
import struct
import time

# MSP command codes
MSP_SET_RAW_RC = 200

def create_msp_command(cmd, data=[]):
    length = len(data)
    checksum = length ^ cmd
    for byte in data:
        checksum ^= byte
    return bytearray([ord('$'), ord('M'), ord('<'), length, cmd] + data + [checksum])

class BetaflightFC:
    def __init__(self, port='/dev/ttyACM0', baudrate=115200):
        self.ser = serial.Serial(port, baudrate, timeout=1)
        print(f"Connected to Betaflight FC on {port}")

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

    def arm_motors(self):
        print("Arming motors...")
        # Set throttle low, yaw right to arm
        self.send_rc_data([1500, 1500, 1000, 2000, 1500, 1500, 1500, 1500])
        time.sleep(2)
        # Set all channels to center, slight throttle
        self.send_rc_data([1500, 1500, 1100, 1500, 1500, 1500, 1500, 1500])

    def disarm_motors(self):
        print("Disarming motors...")
        # Set throttle low, yaw left to disarm
        self.send_rc_data([1500, 1500, 1000, 1000, 1500, 1500, 1500, 1500])

def main():
    fc = BetaflightFC()
    
    # Start by sending neutral RC values
    fc.send_rc_data([1500, 1500, 1000, 1500, 1500, 1500, 1500, 1500])
    
    while True:
        command = input("Enter 'arm', 'disarm', or 'quit': ").strip().lower()
        if command == 'arm':
            fc.arm_motors()
        elif command == 'disarm':
            fc.disarm_motors()
        elif command == 'quit':
            break
        else:
            print("Invalid command")
        
        # Continue sending neutral RC values to prevent RXLOSS
        fc.send_rc_data([1500, 1500, 1000, 1500, 1500, 1500, 1500, 1500])
        time.sleep(0.1)

    fc.ser.close()
    print("Exiting...")

if __name__ == "__main__":
    main()
