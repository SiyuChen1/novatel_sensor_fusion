import numpy as np
from scipy.spatial.transform import Rotation as R

# rotating axes, firstly Y, then X then Z
r1 = R.from_euler('ZXY', [30, 45, 60], degrees=True)
M1 = r1.as_matrix()
q1_value = r1.as_quat()
print(q1_value)
q1 = R.from_quat(q1_value)

r2 = R.from_rotvec(np.array([0.003, -0.048, -0.067]))
M2 = r2.as_matrix()
q2_value = r2.as_quat()
q2 = R.from_quat(q2_value)

print('correct order:')
q_c = q2 * q1
print(q_c.as_quat())

print('wrong order:')
q_w = q1 * q2
print(q_w.as_quat())

error_1 = np.sqrt(np.sum(np.square(q_c.as_quat() - q_w.as_quat())))
error_2 = np.sqrt(np.sum(np.square(q_c.as_quat() + q_w.as_quat())))
error = np.min([error_1, error_2])
error = error * 180 / np.pi
print('calculation error: ', error)
