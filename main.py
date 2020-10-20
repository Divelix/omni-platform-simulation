from omni import Car, vrep
from input import OmniParser
from vis import MyPlotClass, plt
import sys
import threading

vrep.simxFinish(-1)
client_id = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)

if client_id != -1:
    print('Connected to remote API server')
    vrep.simxStartSimulation(client_id, vrep.simx_opmode_blocking)
else:
    sys.exit('Failed connecting to remote API server')

# vrep.simxSynchronous(client_id, True) # call once
# vrep.simxSynchronousTrigger(client_id) # each step
# --------------------------------------------------------------
car_name = 'Car'
wheel_names = ['OmniTL', 'OmniTR', 'OmniBR', 'OmniBL']

w = 0.125
l = 0.125
r = 0.09 / 2  # wheel radius
car = Car(client_id, car_name, wheel_names, w, l, r)
car_thread = threading.Thread(target=car.run)
parser = OmniParser(client_id, car)
input_thread = threading.Thread(target=parser.run)
plotter = MyPlotClass(car)

car_thread.start()
input_thread.start()
plt.show()

input_thread.join()
car_thread.join()

