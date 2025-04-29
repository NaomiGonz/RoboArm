import serial
import argparse
import threading
import time
import json

curr_target = None

def read_serial():
    global curr_target
    while True:
        data = ser.readline().decode('utf-8').strip()
        if data:
            try:
                # Try parsing one JSON at a time
                if '}{' in data:
                    packets = data.split('}{')
                    packets = [packets[0] + '}', '{' + packets[1]]  # simple two-packet split
                    for packet in packets:
                        curr_target = json.loads(packet)
                else:
                    curr_target = json.loads(data)
            except json.JSONDecodeError:
                print(f"Invalid JSON: {data}")

def main():
    global ser
    parser = argparse.ArgumentParser(description='Serial JSON Communication')
    parser.add_argument('port', type=str, help='e.g. /dev/ttyUSB0)')

    args = parser.parse_args()

    ser = serial.Serial(args.port, baudrate=115200, dsrdtr=None)
    ser.setRTS(False)
    ser.setDTR(False)

    serial_recv_thread = threading.Thread(target=read_serial)
    serial_recv_thread.daemon = True
    serial_recv_thread.start()

    try:
        with open("output.txt", "r") as f:
            for line in f:
                command = [float(x) for x in line.split(',')]
                command_str = "{" + f"\"T\":102,\"base\":{command[0]},\"shoulder\":{command[1]},\"elbow\":{command[2]},\"wrist\":{command[3]},\"roll\":{command[4]},\"hand\":3.13,\"spd\":0,\"acc\":1" + "}"
                if not command_str:
                    continue
                #command_str = "{\"T\": 102,\"base\":-1.5,\"shoulder\":1.0,\"elbow\":1.0,\"wrist\":-0.5,\"roll\":1.0,\"hand\":3.13,\"spd\":0,\"acc\":1}"
                #command_str = "{\"T\":102,\"base\":0.5,\"shoulder\":1.0,\"elbow\":1.0,\"wrist\":-0.5,\"roll\":1.0,\"hand\":3.13,\"spd\":1,\"acc\":1}"
                #command_str = "{\"T\":106}"
                ser.write(command_str.encode() + b'\n')
                print(f"sent: {command_str}")
                while True:
                    if curr_target is None:
                        continue  # Wait until we get something
                    try:
                        joint_angles = [curr_target[key] for key in ['b', 's', 'e', 't', 'r', 'g']]
                    except KeyError:
                        print("Received data missing some expected keys.")
                        continue
                    flag = True
                    for i in range(5):  # only first 5 angles, adjust if needed
                        print(f"cmd sent:{command[i]}, reading:{joint_angles[i]}")
                        if abs(command[i] - joint_angles[i]) > 0.1:
                            flag = False
                            break
                    if flag:
                        print("Target reached!")
                        break
                    time.sleep(0.01)  # small sleep to not busy-wait too hard
    except KeyboardInterrupt:
        pass
    except FileNotFoundError:
        print("output.txt not found")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
