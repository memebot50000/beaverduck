import serial
import time
import struct

# MSP command codes
MSP_SET_RAW_RC = 200

def create_msp_command(cmd, data=[]):
    length = len(data)
    checksum = length ^ cmd
    for byte in data:
        checksum ^= byte
    return bytearray([ord('$'), ord('M'), ord('<'), length, cmd] + data + [checksum])

class SpeedybeeFC:
    def __init__(self, port='/dev/ttyACM0', baudrate=115200):
        self.ser = serial.Serial(port, baudrate, timeout=1)
        print(f"Connected to Speedybee FC on {port}")

    def send_msp_command(self, cmd, data=[]):
        command = create_msp_command(cmd, data)
        self.ser.write(command)
        print(f"Sent MSP command: {command.hex()}")

    def send_rc_data(self, channels):
        data = []
        for ch in channels:
            data += [ch & 0xFF, (ch >> 8) & 0xFF]
        self.send_msp_command(MSP_SET_RAW_RC, data)

    def arm_and_spin(self):
        print("Arming motors and spinning slowly...")
        # Set throttle low, yaw right to arm
        self.send_rc_data([1500, 1500, 1000, 2000, 1500, 1500, 1500, 1500])
        time.sleep(2)
        # Set all channels to center, slight throttle
        self.send_rc_data([1500, 1500, 1100, 1500, 1500, 1500, 1500, 1500])

def main():
    fc = SpeedybeeFC()
    
    # Initial delay to ensure FC is ready
    time.sleep(5)
    
    # Arm motors and spin slowly
    fc.arm_and_spin()
    
    # Keep the script running and motors spinning
    try:
        while True:
            fc.send_rc_data([1500, 1500, 1100, 1500, 1500, 1500, 1500, 1500])
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Disarming motors...")
        fc.send_rc_data([1500, 1500, 1000, 1000, 1500, 1500, 1500, 1500])
    
    fc.ser.close()
    print("Exiting...")

if __name__ == "__main__":
    main()
