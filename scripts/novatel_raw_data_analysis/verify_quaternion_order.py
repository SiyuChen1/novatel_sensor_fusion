from novatel_sensor_fusion_py.ultilies.quaternion_multiply import quaternion_multiply
import numpy as np
from scipy.spatial.transform import Rotation as R

r1 = R.from_euler('zxy', [30, 45, 60], degrees=True)
M1 = r1.as_matrix()
q1_value = r1.as_quat()
q1 = R.from_quat(q1_value)
print('q1 type', type(q1))

r2 = R.from_euler('zxy', [60, 30, 45], degrees=True)
M2 = r2.as_matrix()
q2_value = r2.as_quat()
q2 = R.from_quat(q2_value)

M = M2 @ M1
q = R.from_matrix(M).as_quat()
print('using rotation matrix: ', q)

# correct order
q_c = q2 * q1
print('correct order: ', q_c.as_quat())

# wrong order
q_e = q1 * q2
print('wrong order: ', q_e.as_quat())

q_m = quaternion_multiply([q2_value[3], q2_value[0], q2_value[1], q2_value[2]],
                          [q1_value[3], q1_value[0], q1_value[1], q1_value[2]])

q_m = [q_m[1], q_m[2], q_m[3], q_m[0]]

print('multiply with quaternion: ', q_m)
error_1 = np.sqrt(np.sum(np.square(q_c.as_quat() - q_m)))
error_2 = np.sqrt(np.sum(np.square(q_c.as_quat() + q_m)))
error = np.min([error_1, error_2])
error = error * 180 / np.pi
print('calculation error: ', error)
