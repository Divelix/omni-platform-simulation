import vrep
from omnimath import *
import time


class Car:
    is_stop = False
    x_real = [0]
    y_real = [0]
    x_estm = [0]
    y_estm = [0]

    _T = np.eye(4)
    _period = 0.2  # seconds between odometry update

    def __init__(self, client_id, car_name, wheel_names, w, l, r):
        self._next_call = time.time()
        self.client_id = client_id
        assert len(wheel_names) == 4
        _, self.car_handle = vrep.simxGetObjectHandle(client_id, car_name, vrep.simx_opmode_oneshot_wait)
        _, self._position_real = vrep.simxGetObjectPosition(client_id, self.car_handle, -1, vrep.simx_opmode_streaming)

        _, omni_tl_handle = vrep.simxGetObjectHandle(client_id, wheel_names[0], vrep.simx_opmode_oneshot_wait)
        _, omni_tr_handle = vrep.simxGetObjectHandle(client_id, wheel_names[1], vrep.simx_opmode_oneshot_wait)
        _, omni_br_handle = vrep.simxGetObjectHandle(client_id, wheel_names[2], vrep.simx_opmode_oneshot_wait)
        _, omni_bl_handle = vrep.simxGetObjectHandle(client_id, wheel_names[3], vrep.simx_opmode_oneshot_wait)
        self.omni_tl = Wheel(client_id, omni_tl_handle, r)
        self.omni_tr = Wheel(client_id, omni_tr_handle, r)
        self.omni_br = Wheel(client_id, omni_br_handle, r)
        self.omni_bl = Wheel(client_id, omni_bl_handle, r)

        self.w = w
        self.l = l
        self.r = r
        self.H_0 = np.array([[-l-w, 1, -1],
                             [ l+w, 1,  1],
                             [ l+w, 1, -1],
                             [-l-w, 1,  1]])
        self.F = r/4 * np.array([[-1/(l+w), 1/(l+w), 1/(l+w), -1/(l+w)],
                                 [ 1,       1,       1,        1      ],
                                 [-1,       1,      -1,        1      ]])

    def run(self):
        self._next_call = time.time()
        while True:
            self.update_odometry()
            self._next_call += self._period
            now = time.time()
            if now < self._next_call:
                time.sleep(self._next_call - now)
            if self.is_stop:
                break

    def set_wheels_velocities(self, u):
        self.omni_tl.set_velocity(u[0])
        self.omni_tr.set_velocity(u[1])
        self.omni_br.set_velocity(u[2])
        self.omni_bl.set_velocity(u[3])

    def set_car_velocity(self, dq):
        # phi = dq[0]
        # R_phi = np.array([[1,  0,  0],
        #                   [0,  np.cos(phi), np.sin(phi)],
        #                   [0, -np.sin(phi), np.cos(phi)]])
        # H_phi = np.dot(self.H_0, R_phi)
        # u = 1 / self.r * np.dot(H_phi, dq)
        u = 1 / self.r * np.dot(self.H_0, dq)
        self.set_wheels_velocities(u)

    def update_odometry(self):
        q_tl, dq_tl = self.omni_tl.get_angle()
        q_tr, dq_tr = self.omni_tr.get_angle()
        q_br, dq_br = self.omni_br.get_angle()
        q_bl, dq_bl = self.omni_bl.get_angle()
        dq = np.array([dq_tl, dq_tr, dq_br, dq_bl]).T
        v_b = np.dot(self.F, dq)
        v_b6 = np.array([0, 0, v_b[0], v_b[1], v_b[2], 0]).T

        # w = v_b6[:3]
        # v = v_b6[3:].reshape((3, 1))
        # theta = np.linalg.norm(w)  # TODO mistake candidate
        # w_skew = skew(w)
        # e_w = np.eye(3) + np.sin(theta)*w_skew + (1 - np.cos(theta))*w_skew@w_skew  # Rodriguez formula
        # pos = (np.eye(3)*theta + (1 - np.cos(theta))*w_skew + (theta - np.sin(theta))*w_skew@w_skew)@v
        # row_1 = np.concatenate((e_w, pos), axis=1)
        # row_2 = np.array([0, 0, 0, 1]).reshape((1, 4))
        # dT = np.concatenate((row_1, row_2))

        dT = vec6_to_SE3(v_b6)
        # print(dT)
        self._T = self._T @ dT
        self.x_estm.append(self._T[0, 3])
        self.y_estm.append(self._T[1, 3])

        # OPTIONAL: fetch real position from V-Rep for odometry error evaluation
        _, self._position_real = vrep.simxGetObjectPosition(self.client_id, self.car_handle, -1, vrep.simx_opmode_buffer)
        self.x_real.append(self._position_real[0])
        self.y_real.append(self._position_real[1])

    def get_position_error(self):
        real_pos = np.asarray(self._position_real).reshape((3, 1))
        estimated_pos = self._T[:3, 3].reshape((3, 1))
        # estimated_pos[[0, 1], :] = estimated_pos[[1, 0], :]  # body to space frame
        error = np.linalg.norm(real_pos[:2] - estimated_pos[:2])
        return error

    def get_joint_velocities(self):
        v_tl = self.omni_tl.get_joint_velocity()
        v_tr = self.omni_tr.get_joint_velocity()
        v_br = self.omni_br.get_joint_velocity()
        v_bl = self.omni_bl.get_joint_velocity()
        return [v_tl, v_tr, v_br, v_bl]

    def get_rounded_joint_velocities(self):
        vs = self.get_joint_velocities()
        return [round(v, 2) for v in vs]


class Wheel:

    def __init__(self, client_id, handle, radius=0):
        self.client_id = client_id
        self.handle = handle
        self.radius = radius
        _, self.angle = vrep.simxGetJointPosition(self.client_id, self.handle, vrep.simx_opmode_streaming)
        _, self.position = vrep.simxGetObjectPosition(self.client_id, self.handle, -1, vrep.simx_opmode_streaming)
        _, self.orientation = vrep.simxGetObjectOrientation(self.client_id, self.handle, -1, vrep.simx_opmode_streaming)
        _, self.velocity = vrep.simxGetObjectFloatParameter(self.client_id, self.handle, 2012, vrep.simx_opmode_streaming)

    def set_velocity(self, speed):
        vrep.simxSetJointTargetVelocity(self.client_id, self.handle, speed, vrep.simx_opmode_oneshot)

    def get_angle(self):
        _, angle = vrep.simxGetJointPosition(self.client_id, self.handle, vrep.simx_opmode_buffer)
        delta_angle = rotate(angle, self.angle)
        self.angle += delta_angle
        return self.angle, delta_angle

    def get_joint_velocity(self):
        _, self.velocity = vrep.simxGetObjectFloatParameter(self.client_id, self.handle, 2012, vrep.simx_opmode_buffer)
        return self.velocity

    def get_object_position(self):
        _, self.position = vrep.simxGetObjectPosition(self.client_id, self.handle, -1, vrep.simx_opmode_buffer)
        return self.position

    def get_object_orientation(self):
        _, self.orientation = vrep.simxGetObjectOrientation(self.client_id, self.handle, -1, vrep.simx_opmode_buffer)
        return self.orientation
