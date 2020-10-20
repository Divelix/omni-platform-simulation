import vrep


class OmniParser:
    """
    exit - stop simulation and terminate program
    r - set car velocity to zero
    g p - get car position
    g e - get car position error
    g v - get wheel joint velocities
    s v w - set velocity in wheels reference frame (4 numbers)
    s v c - set velocity in car reference frame (3 numbers)
    """
    commands = {
        'quit': 'q',
        'reset': 'r',
        'get': 'g',
        'set': 's',
        'error': 'e',
        'position': 'p',
        'velocity': 'v',
        'car': 'c',
        'wheel': 'w'
    }

    def __init__(self, client_id, car):
        self.client_id = client_id
        self._car = car

    def run(self):
        while True:
            reply = input("Input commands: ")
            words = reply.split(' ')
            if words[0] == self.commands['quit']:
                print("quit")
                vrep.simxStopSimulation(self.client_id, vrep.simx_opmode_oneshot)
                self._car.is_stop = True
                break
            elif words[0] == self.commands['reset']:
                self._car.set_wheels_velocities([0, 0, 0, 0])
            elif words[0] == self.commands['get']:
                print("get", end=" ")
                if words[1] == self.commands['position']:
                    print("position")
                    # print(self.car.get_joint_position())
                elif words[1] == self.commands['velocity']:
                    print("velocity", end=" ")
                    print(self._car.get_joint_velocities())
                elif words[1] == self.commands['error']:
                    print(f"Position error = {self._car.get_position_error()}")
            elif words[0] == self.commands['set']:
                print("set", end=" ")
                if words[1] == self.commands['position']:
                    print("position", end=" ")
                    pos = [0, 0, 0]
                    if ',' in words[-1]:
                        pos = [float(f) for f in words[-1].split(',')]
                    if words[2] == self.commands['car']:
                        self._car.set_car_velocity(pos)
                    elif words[2] == self.commands['wheel']:
                        self._car.set_wheels_velocities(pos)
                    print(pos)
                elif words[1] == self.commands['velocity']:
                    print("velocity", end=" ")
                    vel = [0, 0, 0]
                    if ',' in words[-1]:
                        vel = [float(f) for f in words[-1].split(',')]
                    if words[2] == self.commands['car']:
                        self._car.set_car_velocity(vel)
                    elif words[2] == self.commands['wheel']:
                        self._car.set_wheels_velocities(vel)
                    print(vel)
