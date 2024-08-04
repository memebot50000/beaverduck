#!/bin/bash

# Update and install necessary packages
sudo apt-get update
sudo apt-get install -y python3-pip python3-serial
pip3 install pyserial

# Create directory for our manual control scripts
mkdir -p ~/manual_control
cd ~/manual_control

# Create spektrum_receiver.py
cat > spektrum_receiver.py << EOL
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
EOL
chmod +x spektrum_receiver.py

# Create speedybee_comm.py
cat > speedybee_comm.py << EOL
#!/usr/bin/env python3

import serial
import time

class SpeedybeeComm:
    def __init__(self):
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

    def send_data(self, channels):
        formatted_data = bytearray([0x55, 0xAA])  # Start bytes
        for ch in channels:
            formatted_data += ch.to_bytes(2, byteorder='little')
        formatted_data += bytearray([0x00, 0x00])  # End bytes
        self.ser.write(formatted_data)

    def run(self):
        try:
            while True:
                # For testing, send dummy data
                dummy_channels = [1000, 1500, 2000, 1750]
                self.send_data(dummy_channels)
                print(f"Sent data: {dummy_channels}")
                time.sleep(0.02)  # 50Hz update rate
        except KeyboardInterrupt:
            pass
        finally:
            self.ser.close()

if __name__ == '__main__':
    comm = SpeedybeeComm()
    comm.run()
EOL
chmod +x speedybee_comm.py

# Configure UART (if not already configured)
if ! grep -q "enable_uart=1" /boot/config.txt; then
    echo "enable_uart=1" | sudo tee -a /boot/config.txt
fi
sudo sed -i '/console=serial0,115200/d' /boot/cmdline.txt

echo "Setup complete. To run the scripts, use the following commands in separate terminals:"
echo "python3 ~/manual_control/spektrum_receiver.py"
echo "python3 ~/manual_control/speedybee_comm.py"
echo "Note: You may need to reboot for UART changes to take effect."
