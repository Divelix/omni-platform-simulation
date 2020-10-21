import numpy as np


def rotate(angle, desired_angle):
    xa = [np.cos(angle), np.sin(angle)]
    rot = np.array([[np.cos(-desired_angle), -np.sin(-desired_angle)], [np.sin(-desired_angle), np.cos(-desired_angle)]])
    delta_v = np.dot(rot, xa)
    delta = np.arctan2(delta_v[1], delta_v[0])
    return delta


def is_near_zero(s):
    return abs(s) < 1e-6


def normalize(v):
    return v / np.linalg.norm(v)


def skew(v):
    v = np.ravel(v)
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])


def vec6_to_se3(v):
    return np.r_[np.c_[skew(v[:3]), v[3:]], [[0, 0, 0, 1]]]


def vec6_to_SE3(v):
    se3 = vec6_to_se3(v)
    return se3_to_SE3(se3)


def so3_to_vec(so3):
    return np.array([so3[2, 1], so3[0, 2], so3[1, 0]])


def w_split(w):
    return normalize(w), np.linalg.norm(w)


def so3_to_SO3(so3):
    w_theta = so3_to_vec(so3)
    if is_near_zero(np.linalg.norm(w_theta)):
        return np.eye(3)
    else:
        theta = w_split(w_theta)[1]
        w_skew = so3 / theta
        return np.eye(3) + np.sin(theta)*w_skew + (1 - np.cos(theta))*w_skew@w_skew  # Rodriguez formula


def se3_to_SE3(se3):
    w_theta = so3_to_vec(se3[0:3, 0:3])
    if is_near_zero(np.linalg.norm(w_theta)):
        return np.r_[np.c_[np.eye(3), se3[0:3, 3]], [[0, 0, 0, 1]]]
    else:
        theta = w_split(w_theta)[1]
        w_skew = se3[0:3, 0:3] / theta
        return np.r_[
                np.c_[
                    so3_to_SO3(se3[0:3, 0:3]),
                    (np.eye(3) * theta + (1 - np.cos(theta)) * w_skew + (theta - np.sin(theta)) * w_skew @ w_skew) @ se3[0:3, 3] / theta
                ],
                [[0, 0, 0, 1]]
        ]
