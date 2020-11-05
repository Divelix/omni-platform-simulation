#!/usr/bin/env python3
import serial
import time

port = serial.Serial('/dev/ttyACM0', 115200)
port.flush()

while True:
    # if port.in_waiting > 0:
    start = time.time()
    port.write(b"1.1 2.2 3.3 4.4\n")
    line = port.readline().decode('utf-8').rstrip()
    rpms = [float(rpm) for rpm in line.split(' ')]
    print(f"sum = {sum(rpms)}| dt = {time.time() - start} secs")
    # print(line)
    # time.sleep(0.1)