import serial
import struct
import time

MSP_SET_RAW_RC = 200

def msp_encode(cmd, data):
    size = len(data)
    total = 0
    for byte in data:
        total ^= byte
    frame = [ord('$'), ord('M'), ord('<'), size, cmd]
    frame.extend(data)
    frame.append(total & 0xFF)
    return bytes(frame)

class BetaflightFC:
    def __init__(self, port='/dev/ttyACM0', baudrate=115200):
        self.ser = serial.Serial(port, baudrate, timeout=1)
        print(f"Connected to Betaflight FC on {port}")

    def send_rc_data(self, channels):
        data = []
        for ch in channels:
            data.append(ch & 0xFF)
            data.append((ch >> 8) & 0xFF)
        packet = msp_encode(MSP_SET_RAW_RC, data)
        self.ser.write(packet)
        print(f"Sent RC data: {channels}")

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
