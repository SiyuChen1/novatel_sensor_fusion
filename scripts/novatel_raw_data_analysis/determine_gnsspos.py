#!/usr/bin/env python3

# this script used to determine the recorded bestgnss/bestpos log is the position
# of IMU body or the first antenna
import argparse
import matplotlib.pyplot as plt
import os
import pymap3d as pm
from pathlib import Path


def get_data(abs_path, file_name, lla_ref):
    e_l = []
    n_l = []
    u_l = []
    with open(abs_path / file_name) as data_file:
        content = data_file.read().split('\n')
        for index, line in enumerate(content):
            # if "BESTGNSSPOSA" in line:
            if 'BESTPOSA' in line:
                msg = line.split(';')
                data = msg[1].split(',')
                lat = float(data[2])
                lon = float(data[3])
                h = float(data[4])
                e, n, u = pm.geodetic2enu(
                    lat, lon, h, lla_ref[0], lla_ref[1], lla_ref[2])
                e_l.append(e)
                n_l.append(n)
                u_l.append(u)
    return e_l, n_l, u_l


abs_path = Path(os.path.abspath(__file__))

parser = argparse.ArgumentParser(description="Determine bestgnss/gnss position, IMU body or Antenna 1?")
parser.add_argument("--folder-path", type=str,
                    default=str(abs_path.parent.parent.parent / 'data/gnss_imu_logs/determine_gnss_pos'))

# Parse the arguments
args = parser.parse_args()
folder_path = args.folder_path
# folder_path = '/home/siyuchen/catkin_ws/src/novatel_sensor_fusion/data/gnss_imu_logs/determine_gnss_pos'
# folder_path = '../../data/gnss_imu_logs/determine_gnss_pos'

file_names = ['Rotation um Antenne1_2023-07-03_14-36-08.log',
              'Rotation um IMU_2023-07-03_14-36-47.log']
labels = ['um Antenna', 'um IMU']

# set up reference lla values
lat0, lon0, h0 = 50.77737927959, 6.07872928972, 177.8415
ref_lla = [lat0, lon0, h0]

for i, f in enumerate(file_names):
    e_l, n_l, _ = get_data(Path(folder_path), f, ref_lla)
    plt.plot(e_l, n_l, '*', label=labels[i])

# after visualisation, it can be determined that the gnss measurements are positions
# of the first antenna
plt.legend()
plt.axis('equal')
plt.xlabel('East m')
plt.xlabel('North m')
plt.show()
