#!/usr/bin/env python3

import serial
import time

class SpektrumReceiver:
    def __init__(self):
        self.ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=1)

    def read_spektrum(self):
        while True:
            if self.ser.in_waiting >= 16:
                data = self.ser.read(16)
                if data[0] == 0x0F and data[1] == 0xA2:
                    channels = []
                    for i in range(4):  # Only read 4 channels
                        ch = (data[2*i+3] << 8) | data[2*i+2]
                        channels.append(ch & 0x07FF)
                    print(f"Channel values: {channels}")
            time.sleep(0.02)  # 50Hz update rate

    def run(self):
        try:
            self.read_spektrum()
        except KeyboardInterrupt:
            pass
        finally:
            self.ser.close()

if __name__ == '__main__':
    receiver = SpektrumReceiver()
    receiver.run()
