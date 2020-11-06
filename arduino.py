import serial
import time
from threading import Thread


class ArduinoTalker(Thread):

    def __init__(self):
        Thread.__init__(self)
        self.serial = serial.Serial('/dev/ttyACM0', 115200)
        self.serial.flush()

    def transmit(self, u):
        assert len(u) == 4
        msg = ' '.join(map(str, u)) + '\n'
        self.serial.write(msg)
        # self.serial.write(b"1.1 2.2 3.3 4.4\n")

    def receive(self):
        msg = self.serial.readline().decode('utf-8').rstrip()
        wheels_feedback = [float(rpm) for rpm in msg.split(' ')]
        assert len(wheels_feedback) == 4
        return wheels_feedback
