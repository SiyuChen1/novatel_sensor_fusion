import numpy as np
import transforms3d

angle_y = 0.105304289
angle_x = 2.745404166
azimuth = 182.376753587

w = 0.020711630
x = -0.000421657
y = 0.023969770
z = 0.999498024
q = np.array([w, x, y, z])

print('norm of the quaternion', np.sum(np.square(q)))
M = transforms3d.quaternions.quat2mat(q)
# rotation order firstly z, then x, at last y
angles = np.array(transforms3d.euler.mat2euler(M, 'syxz'))
print(transforms3d.axangles.mat2axangle(M))

ax = float(angle_x) * np.pi / 180
ay = float(angle_y) * np.pi / 180
az = - float(azimuth) * np.pi / 180
# Quaternion in w, x, y z (real, then vector) format
q_t = transforms3d.euler.euler2quat(ay, ax, az, 'syxz')
M_t = transforms3d.quaternions.quat2mat(q_t)
print(transforms3d.axangles.mat2axangle(M_t))

print(angles * 180 / np.pi)
