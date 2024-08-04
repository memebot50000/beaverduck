#!/bin/bash

# Update and install necessary packages
sudo apt-get update
sudo apt-get install -y python3-pip python3-serial ros-noetic-rosserial-python
pip3 install pyserial

# Create new ROS workspace
mkdir -p ~/manual_control_ws/src
cd ~/manual_control_ws/src

# Create and set up spektrum_receiver package
catkin_create_pkg spektrum_receiver rospy std_msgs
cd spektrum_receiver/src
cat > spektrum_node.py << EOL
#!/usr/bin/env python3

import rospy
import serial
from std_msgs.msg import Int16MultiArray

class SpektrumReceiver:
    def __init__(self):
        rospy.init_node('spektrum_receiver', anonymous=True)
        self.pub = rospy.Publisher('rc_channels', Int16MultiArray, queue_size=10)
        self.ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=1)
        self.rate = rospy.Rate(50)  # 50Hz update rate

    def read_spektrum(self):
        while not rospy.is_shutdown():
            if self.ser.in_waiting >= 16:
                data = self.ser.read(16)
                if data[0] == 0x0F and data[1] == 0xA2:
                    channels = []
                    for i in range(4):  # Only read 4 channels
                        ch = (data[2*i+3] << 8) | data[2*i+2]
                        channels.append(ch & 0x07FF)
                    
                    msg = Int16MultiArray()
                    msg.data = channels
                    self.pub.publish(msg)
            
            self.rate.sleep()

    def run(self):
        try:
            self.read_spektrum()
        except rospy.ROSInterruptException:
            pass
        finally:
            self.ser.close()

if __name__ == '__main__':
    receiver = SpektrumReceiver()
    receiver.run()
EOL
chmod +x spektrum_node.py
cd ..
mkdir launch
cat > launch/spektrum_receiver.launch << EOL
<launch>
  <node name="spektrum_receiver" pkg="spektrum_receiver" type="spektrum_node.py" output="screen"/>
</launch>
EOL

# Create and set up speedybee_comm package
cd ~/manual_control_ws/src
catkin_create_pkg speedybee_comm rospy std_msgs
cd speedybee_comm/src
cat > speedybee_node.py << EOL
#!/usr/bin/env python3

import rospy
import serial
from std_msgs.msg import Int16MultiArray

class SpeedybeeComm:
    def __init__(self):
        rospy.init_node('speedybee_comm', anonymous=True)
        self.sub = rospy.Subscriber('rc_channels', Int16MultiArray, self.callback)
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        self.rate = rospy.Rate(50)  # 50Hz update rate

    def callback(self, data):
        if len(data.data) == 4:  # Expecting 4 channels from AR410
            # Format the data as needed for the Speedybee FC
            formatted_data = bytearray([0x55, 0xAA])  # Start bytes
            for ch in data.data:
                formatted_data += ch.to_bytes(2, byteorder='little')
            formatted_data += bytearray([0x00, 0x00])  # End bytes
            self.ser.write(formatted_data)

    def run(self):
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            pass
        finally:
            self.ser.close()

if __name__ == '__main__':
    comm = SpeedybeeComm()
    comm.run()
EOL
chmod +x speedybee_node.py
cd ..
mkdir launch
cat > launch/speedybee_comm.launch << EOL
<launch>
  <node name="speedybee_comm" pkg="speedybee_comm" type="speedybee_node.py" output="screen"/>
</launch>
EOL

# Build the workspace
cd ~/manual_control_ws
catkin_make

# Source the new workspace
source devel/setup.bash

# Configure UART (if not already configured)
if ! grep -q "enable_uart=1" /boot/config.txt; then
    echo "enable_uart=1" | sudo tee -a /boot/config.txt
fi
sudo sed -i '/console=serial0,115200/d' /boot/cmdline.txt

# Create a launch file for both nodes
cd ~/manual_control_ws/src
mkdir -p manual_control/launch
cat > manual_control/launch/manual_control.launch << EOL
<launch>
  <node name="spektrum_receiver" pkg="spektrum_receiver" type="spektrum_node.py" output="screen"/>
  <node name="speedybee_comm" pkg="speedybee_comm" type="speedybee_node.py" output="screen"/>
</launch>
EOL

echo "Setup complete. To run the nodes, use the following command:"
echo "roslaunch manual_control manual_control.launch"
echo "Note: You may need to reboot for UART changes to take effect."
