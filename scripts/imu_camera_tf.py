import numpy as np
from scipy.spatial.transform import Rotation as R

# see email on 20.11.2023 at 15:30 from Effkemann

# imu (not imu body frame, but vehicle frame): x to right, y forward, z points up
# left cam: x to right, y points up, z backward
# right cam: x to right, y points up, z backward

# from imu to left camera
imu_cam_left = np.zeros((3, 3))
imu_cam_left[:, 0] = [0.999258, 0.000897, 0.038500]
imu_cam_left[:, 1] = [-0.038439, -0.037827, 0.998545]
imu_cam_left[:, 2] = [0.002352, -0.999284, -0.037765]
# print("rotation imu_cam_left: \n", imu_cam_left)
# print(np.linalg.det(imu_cam_left))

tf_imu_cam_left = np.zeros((4, 4))
tf_imu_cam_left[0:3, 0:3] = imu_cam_left
tf_imu_cam_left[0:3, 3] = 1e-3 * np.array([-458.9, 92.6, 170.6])
tf_imu_cam_left[3, 3] = 1
print("tf imu_cam_left: \n", tf_imu_cam_left)

# from imu to right camera
imu_cam_right = np.zeros((3, 3))
imu_cam_right[:, 0] = [0.995794, 0.023624, 0.088518]
imu_cam_right[:, 1] = [-0.088152, -0.016090, 0.995977]
imu_cam_right[:, 2] = [0.024953, -0.999591, -0.013940]
# print("rotation imu_cam_right: \n", imu_cam_right)
# print(np.linalg.det(imu_cam_right))

tf_imu_cam_right = np.zeros((4, 4))
tf_imu_cam_right[0:3, 0:3] = imu_cam_right
tf_imu_cam_right[0:3, 3] = 1e-3 * np.array([451.1, 114.4, 190.4])
tf_imu_cam_right[3, 3] = 1
# print("tf imu_cam_right: \n", tf_imu_cam_right)

# from left camera to right camera
# calculated by multiple camera calibration
cam_left_cam_right = np.zeros((3, 3))
cam_left_cam_right[:, 0] = [0.998532, -0.022429, -0.049303]
cam_left_cam_right[:, 1] = [0.021223, 0.999463, -0.024892]
cam_left_cam_right[:, 2] = [0.049837, 0.023810, 0.998472]

tf_cam_left_cam_right = np.zeros((4, 4))
tf_cam_left_cam_right[0:3, 0:3] = cam_left_cam_right
tf_cam_left_cam_right[0:3, 3] = 1e-3 * np.array([910.1, -16.0, -20.5])
tf_cam_left_cam_right[3, 3] = 1
q_real = R.from_matrix(cam_left_cam_right).as_quat()
# print("tf cam_left_cam_right: \n", tf_cam_left_cam_right)
# print(tf_cam_left_cam_right[0:3, 3])

# calculate transform from left cam to right cam by tf_imu_left_cam and
# tf_imu_right_cam
inv_tf_imu_cam_left = np.zeros((4, 4))
inv_tf_imu_cam_left[0:3, 0:3] = imu_cam_left.T
inv_tf_imu_cam_left[0:3, 3] = -imu_cam_left.T @ tf_imu_cam_left[0:3, 3]
inv_tf_imu_cam_left[3, 3] = 1
estimated_tf_cam_left_cam_right = inv_tf_imu_cam_left @ tf_imu_cam_right
q_estimated = R.from_matrix(estimated_tf_cam_left_cam_right[0:3, 0:3]).as_quat()
# print(estimated_tf_cam_left_cam_right[0:3, 0:3])
# print(estimated_tf_cam_left_cam_right[0:3, 3])

# calculate difference between two rotation matrix
# method 1: Norm of the Difference of Quaternions
# https://www.cs.cmu.edu/~cga/dynopt/readings/Rmetric.pdf page 4, section 3.3
# and equation 31
error_1 = np.sqrt(np.sum(np.square(q_real - q_estimated)))
error_2 = np.sqrt(np.sum(np.square(q_real + q_estimated)))
error = np.min([error_1, error_2])
# using equation 31
error = np.arccos(1 - error ** 2 / 2)
error = error * 180 / np.pi
print("rotation error in degree", error)

# convert camera frame to orb slam3 convention
# in orb slam3, z forward, x to right, y points down
# which can be done by multiply np.diag([1, -1, -1, 1])
tf_cam_left_cam_right_orb = \
    np.linalg.inv(tf_imu_cam_left @ np.diag([1, -1, -1, 1])) \
    @ tf_imu_cam_right @ np.diag([1, -1, -1, 1])
print(tf_cam_left_cam_right_orb)

# using multiple camera calibration
tf_cam_left_cam_right_orb_2 = \
    np.linalg.inv(np.diag([1, -1, -1, 1])) \
    @ tf_cam_left_cam_right @ np.diag([1, -1, -1, 1])
print(tf_cam_left_cam_right_orb_2)

print("tf imu_cam_left in orb convention: \n", tf_imu_cam_left @ np.diag([1, -1, -1, 1]))
