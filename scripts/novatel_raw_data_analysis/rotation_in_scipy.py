import numpy as np
from scipy.spatial.transform import Rotation as R

angle_y = 1.408718133
angle_x = -0.429207488
azimuth = 141.058265204

w = 0.333256585
x = 0.010341571
y = 0.007628655
z = -0.942748590

# angle_y = 0.105304289
# angle_x = 2.745404166
# azimuth = 182.376753587
#
# w = 0.020711630
# x = -0.000421657
# y = 0.023969770
# z = 0.999498024

q = np.array([x, y, z, w])
r1 = R.from_quat(q)
M1 = r1.as_matrix()

ax = float(angle_x) * np.pi / 180
ay = float(angle_y) * np.pi / 180
az = - float(azimuth) * np.pi / 180
r2 = R.from_euler('zxy', [az, ax, ay], degrees=False)
M2 = r2.as_matrix()

r3 = R.from_euler('zxy', [-az, -ax, -ay], degrees=False)
M3 = r3.as_matrix()

r4 = R.from_euler('yxz', [-ay, -ax, -az], degrees=False)
M4 = r4.as_matrix()

r5 = R.from_euler('ZXY', [az, ax, ay], degrees=False)
M5 = r5.as_matrix()

r6 = R.from_euler('yxz', [ay, ax, az], degrees=False)
M6 = r6.as_matrix()

print(M3 @ M1)
print(M3 @ M5)
print(M4 @ M2)
print(np.abs(M1 - M5))
print(np.abs(M1 - M6))

Rx = R.from_euler('x', 45, degrees=True).as_matrix()
Ry = R.from_euler('y', 30, degrees=True).as_matrix()
Rz = R.from_euler('z', 60, degrees=True).as_matrix()

# rotation order firstly x, then y, at last z, rotatimg
R6 = R.from_euler('ZYX', [60, 30, 45], degrees=True).as_matrix()

# rotation order firstly z, then y, at last x, fixed rotation axes
R7 = R.from_euler('xyz', [45, 30, 60], degrees=True).as_matrix()
# print(np.abs(R6 - Rz @ Ry @ Rx))
print(np.allclose(R6, Rz @ Ry @ Rx))
# print(np.abs(R7 - Rz @ Ry @ Rx))
print(np.allclose(R7, Rz @ Ry @ Rx))
