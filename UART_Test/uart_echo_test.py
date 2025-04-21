import serial
import time

# Open UART1 (/dev/ttyS1 maps to P9_24/P9_26)
ser = serial.Serial('/dev/ttyS1', baudrate=115200, timeout=1)

while True:
    # Send test input: 3 fixed-point joint angles
    tx_data = b'45,30,15\n'
    print("Sending:", tx_data)
    ser.write(tx_data)

    # Wait for FPGA to respond
    rx_data = ser.readline()
    if rx_data:
        print("Received:", rx_data.decode().strip())

    time.sleep(1)
