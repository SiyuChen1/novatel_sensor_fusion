import matplotlib.pyplot as plt
import numpy as np
import pymap3d as pm


def read_camera_path(filepath):
    transformations = []
    with open(filepath, 'r') as file:
        for line in file:
            # Splitting the line by spaces and converting the parts to floats
            parts = line.split()
            numbers = [float(part) for part in parts]

            transformations.append(numbers)
        transformations = np.array(transformations).reshape(-1, 3, 4)
        N = transformations.shape[0]
        sub_arr = np.array([[[0, 0, 0, 1]]] * N )
        transformations = np.concatenate((transformations, sub_arr), axis=1)

    return transformations


def get_data(abs_path, file_name):
    ref_is_set = False
    e_l, n_l, u_l = []
    lat0, lon0, h0 = 0
    with open(abs_path + file_name) as data_file:
        content = data_file.read().split('\n')
        for index, line in enumerate(content):
            if 'BESTPOSA' in line:
                msg = line.split(';')
                data = msg[1].split(',')
                lat = float(data[2])
                lon = float(data[3])
                h = float(data[4])
                if not ref_is_set:
                    lat0 = lat
                    lon0 = lon
                    h0 = h
                    ref_is_set = True
                e, n, u = pm.geodetic2enu(lat, lon, h, lat0, lon0, h0)
                e_l.append(e)
                n_l.append(n)
                u_l.append(u)
    return e_l, n_l, u_l


camera_pose_path = '/home/siyuchen/lib/ORB_SLAM3/Examples/CameraTrajectory.txt'
left_camera = np.array([-458.0, 46.3, 497.3])
antenna_1 = np.array([-525.6, 5.0, 553.5])
Ta_lc = left_camera - antenna_1
print(Ta_lc)
Ta_lc = Ta_lc / 1000

Ra_lc = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]])

TFa_lc = np.zeros((4, 4))
TFa_lc[0:3, 0:3] = Ra_lc
TFa_lc[0:3, 3] = Ta_lc
TFa_lc[3, 3] = 1
print(TFa_lc)

camera_traj = read_camera_path(camera_pose_path)
camera_traj = camera_traj.transpose(1, 2, 0)
camera_traj = TFa_lc @ camera_traj


