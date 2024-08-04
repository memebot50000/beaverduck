#!/usr/bin/env python3

import serial
import time
import glob
import threading
import os
import subprocess

def check_uart_config():
    try:
        config = subprocess.check_output(['vcgencmd', 'get_config', 'uart0']).decode('utf-8')
        print("UART Configuration:")
        print(config)
    except Exception as e:
        print(f"Error checking UART configuration: {e}")

def check_gpio_config():
    try:
        with open('/sys/kernel/debug/pinctrl/3f200000.gpio/pins') as f:
            pins = f.read()
        print("GPIO Pin Configuration:")
        print(pins)
    except Exception as e:
        print(f"Error checking GPIO configuration: {e}")

def uart_loopback_test():
    try:
        ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=1)
        test_data = b'Hello, UART!'
        ser.write(test_data)
        time.sleep(0.1)
        received = ser.read(len(test_data))
        if received == test_data:
            print("UART Loopback Test Passed!")
        else:
            print(f"UART Loopback Test Failed. Sent: {test_data}, Received: {received}")
        ser.close()
    except Exception as e:
        print(f"Error during UART loopback test: {e}")

# ... (SpektrumReceiver and SpeedybeeComm classes remain unchanged)

if __name__ == '__main__':
    # Print system information
    print("System Information:")
    print(f"Python version: {os.sys.version}")
    print(f"Operating system: {os.name}")
    print(f"Platform: {os.sys.platform}")
    
    print("\nAdditional System Information:")
    print("UART configuration in /boot/config.txt:")
    os.system("grep uart /boot/config.txt")
    
    print("\nGPIO Information:")
    check_gpio_config()
    
    print("\nUART Configuration:")
    check_uart_config()
    
    # Check if ttyAMA0 exists
    if os.path.exists('/dev/ttyAMA0'):
        print("\nttyAMA0 exists")
    else:
        print("\nttyAMA0 does not exist")
    
    # List all tty devices
    tty_devices = glob.glob('/dev/tty*')
    print("\nAvailable tty devices:")
    for device in tty_devices:
        print(device)


    spektrum_receiver = SpektrumReceiver()
    speedybee_comm = SpeedybeeComm()

    print("\nStarting Spektrum receiver and Speedybee communication...")
    # Run SpektrumReceiver in a separate thread
    receiver_thread = threading.Thread(target=spektrum_receiver.read_spektrum)
    receiver_thread.start()

    # Run SpeedybeeComm in the main thread
    try:
        speedybee_comm.run(spektrum_receiver)
    except KeyboardInterrupt:
        print("\nScript terminated by user.")
    finally:
        print("Cleaning up...")
        # You might want to add any necessary cleanup code here
