#!/usr/bin/env python3
from omni import *
from arduino import *
import time
import threading

car = Car()

talker = ArduinoTalker()
talker.start()

talker.join()
