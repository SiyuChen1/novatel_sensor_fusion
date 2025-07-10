import numpy as np
from scipy.spatial.transform import Rotation as R

# # rotation matrix from Christoph Effkemann on 23.02.2024 14:37
# # 20240223_Stereo_RelativeOrientation.xlsx
rotation_left_right = np.zeros((3, 3))
rotation_left_right[0, :] = [0.999103, -0.006228, 0.041905]
rotation_left_right[1, :] = [0.004933, 0.999508, 0.030940]
rotation_left_right[2, :] = [-0.042078, -0.030708, 0.998643]

# # rotation matrix from Christoph Effkemann on 17.11.2023 13:51
# # 20240223_Stereo_RelativeOrientation.xlsx
# rotation_left_right = np.zeros((3, 3))
# rotation_left_right[0, :] = [0.998532, 0.021223, 0.049837]
# rotation_left_right[1, :] = [-0.022429, 0.999463, 0.023810]
# rotation_left_right[2, :] = [-0.049303, -0.024892, 0.998472]

rotation_christoph_orbslam = R.from_euler('x', 180, degrees=True)
rotation_left_right_orb = \
    rotation_christoph_orbslam.as_matrix().transpose() @ rotation_left_right \
    @ rotation_christoph_orbslam.as_matrix()

print(rotation_left_right_orb)
