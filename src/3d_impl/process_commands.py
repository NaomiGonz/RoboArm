import serial
import argparse
import threading
import time

def read_serial():
    while True:
        data = ser.readline().decode('utf-8')
        # if data:
        #     print(f"Received: {data}", end='')

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
                command = line.strip()
                if not command:
                    continue
                ser.write(command.encode() + b'\n')
                print(f"sent: {command}")
                time.sleep(10)
    except KeyboardInterrupt:
        pass
    except FileNotFoundError:
        print("output.txt not found")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
