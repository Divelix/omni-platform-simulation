from omnimath import *

w = np.array([1, 2, 3]).reshape(3, 1)
v = np.array([4, 5, 6]).reshape(3, 1)
twist = np.r_[w, v]
se3 = vec6_to_SE3(twist)
print(se3)
