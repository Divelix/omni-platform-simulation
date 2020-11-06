from omniSim import CarSim, vrep
from input import OmniParser
from vis import OmniPlotter, plt
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
car = CarSim(client_id, car_name, wheel_names, w, l, r)
car_thread = threading.Thread(target=car.run)
car_thread.start()

parser = OmniParser(client_id, car)
input_thread = threading.Thread(target=parser.run)
input_thread.start()

plotter = OmniPlotter(car)
plt.show()

car_thread.join()
input_thread.join()

