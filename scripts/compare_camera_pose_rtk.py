import re
import sys
import math
from pathlib import Path
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import numpy as np
import pymap3d as pm
from gps_time import GPSTime
from novatel_sensor_fusion_py.lla2geodetic.lla2geodetic import lla2enu


class Vector3D:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z


class IMUMessage:
    def __init__(self, ts, angular_velocity, linear_acceleration):
        self.time_stamp = ts
        self.angular_velocity = angular_velocity
        self.linear_acceleration = linear_acceleration


class LLAMessage:
    def __init__(self, ts, sol_stat, pos_type, lla_with_std):
        self.time_stamp = ts
        self.sol_stat = sol_stat
        self.pos_type = pos_type
        self.lla_with_std = lla_with_std


def gps_time_to_posix_time(gps_week, seconds_in_week):
    current_gps_time = GPSTime(week_number=int(gps_week), time_of_week=float(seconds_in_week))
    # convert gps time to python datetime and then to posix time
    current_posix_time = current_gps_time.to_datetime().timestamp()
    return current_posix_time


def round_float_in_scientific_notation(num, up=True):
    coeff, base = str(num).split('e')
    if up:
        coeff = math.ceil(float(coeff))
    else:
        coeff = round(float(coeff))
    return coeff * 10 ** float(base)


def is_scientific_notation(s):
    # This pattern matches strings in scientific notation.
    # It checks for optional leading sign (+/-), digits before and after the decimal point,
    # optional decimal point and fractional part, followed by 'e' or 'E',
    # an optional sign for the exponent, and one or more digits for the exponent.
    pattern = r'^[+-]?(\d+(\.\d+)?|\.\d+)[eE][+-]?\d+$'

    return re.match(pattern, s) is not None


def read_camera_trajectory(filepath):
    transformations = []
    with open(filepath, 'r') as file:
        for line in file:
            # Splitting the line by spaces and converting the parts to floats
            parts = line.split()
            numbers = [float(part) for part in parts]

            transformations.append(numbers)
        transformations = np.array(transformations).reshape(-1, 3, 4)
        N = transformations.shape[0]
        sub_arr = np.array([[[0, 0, 0, 1]]] * N)
        transformations = np.concatenate((transformations, sub_arr), axis=1)

    return transformations


def read_camera_timestamp(filepath):
    timestamps = []
    data_type = [('real_ts', np.float32), ('desired_ts', np.float32), ('id', np.int32)]
    with open(filepath, 'r') as file:
        for line in file:
            # Splitting the line by spaces and converting the parts to floats
            parts = line.split()

            # first column: time when the image recorded
            # second column: desired time stamp
            # third column: image id
            ts = [float(part) for part in parts]
            ts[-1] = int(ts[-1])
            timestamps.append(tuple(ts))
    return np.array(timestamps, dtype=data_type)


def get_gps_imu_data(abs_path, file_name):
    lla_with_timestamp = []
    quaternion_with_timestamp = []
    imu_with_timestamp = []
    imu_data_rate = 100
    with open(abs_path / file_name) as data_file:
        content = data_file.read().split('\n')
        for index, line in enumerate(content):
            if 'BESTPOSA' in line:
                msg = line.split(';')
                header = msg[0].split(',')
                # get posix time
                gps_week = int(header[5])
                seconds_in_week = float(header[6])
                current_posix_time = gps_time_to_posix_time(gps_week, seconds_in_week)

                data = msg[1].split(',')

                sol_stat = str(data[0])
                pos_type = str(data[1])

                # latitude, longitude, altitude, std of latitude, std of longitude, std of altitude
                lat = float(data[2])
                lon = float(data[3])
                alt = float(data[4])

                lat_std = float(data[7])
                long_std = float(data[8])
                alt_std = float(data[9])

                lla_msg = LLAMessage(
                    ts=current_posix_time,
                    pos_type=pos_type,
                    sol_stat=sol_stat,
                    lla_with_std=[lat, lon, alt, lat_std, long_std, alt_std]
                )

                lla_with_timestamp.append(lla_msg)

            if 'INSATTQSA' in line:
                msg = line.split(';')
                data = msg[1].split(',')
                gps_week = int(data[0])
                seconds_in_week = float(data[1])
                w = float(data[2])
                x = float(data[3])
                y = float(data[4])
                z = float(data[5])
                quaternion_with_timestamp.append([gps_week, seconds_in_week, w, x, y, z])
            if 'CORRIMUSA' in line:
                msg = line.split(';')
                header = msg[0].split(',')
                data = msg[1].split(',')
                gps_week = int(header[1])
                seconds_in_week = float(header[2])
                imu_data_count = int(data[0])
                if imu_data_count > 0:
                    ts = gps_time_to_posix_time(gps_week, seconds_in_week)

                    angular_velocity = Vector3D(
                        x=imu_data_rate / imu_data_count * float(data[1]),
                        y=imu_data_rate / imu_data_count * float(data[2]),
                        z=imu_data_rate / imu_data_count * float(data[3])
                    )

                    linear_acceleration = Vector3D(
                        x=imu_data_rate / imu_data_count * float(data[4]),
                        y=imu_data_rate / imu_data_count * float(data[5]),
                        z=imu_data_rate / imu_data_count * float(data[6])
                    )

                    imu_msg = IMUMessage(
                        ts=ts,
                        angular_velocity=angular_velocity,
                        linear_acceleration=linear_acceleration
                    )
                    imu_with_timestamp.append(imu_msg)

    return lla_with_timestamp,\
        quaternion_with_timestamp, \
        imu_with_timestamp


def read_config(filepath):
    with open(filepath) as f:
        for line in f:
            if 'SETINSTRANSLATION' in line:
                print(line)


dataset_dir = Path('/home/siyuchen/Documents/20231031/20231031_105114')
slam_method = 'stella'
experiment_id = 'experiment1'

# read start_id used in slam
start_id = None
with open(dataset_dir / 'slam' / slam_method / experiment_id / 'start_id.txt', 'r') as file:
    for line in file:
        start_id = int(line)

# read IMU and GPS data
imu_file_name = 'IMUData_20231031_105114.log'
lla_with_timestamp, quaternion_with_timestamp, \
    imu_with_timestamp = get_gps_imu_data(dataset_dir, imu_file_name)


lla_list = np.array([lla_msg.lla_with_std for lla_msg in lla_with_timestamp])
time_stamp_list = np.array([lla_msg.time_stamp for lla_msg in lla_with_timestamp])
quaternion_with_timestamp = np.array(quaternion_with_timestamp)
print('quaternion_with_timestamp.shape', quaternion_with_timestamp.shape)

# # read camera timestamp
# camera_timestamp_name = 'Stereocam_20231031_105114.log'
# camera_timestamp = read_camera_timestamp(dataset_dir / camera_timestamp_name)
# print(camera_timestamp.dtype)
# print('camera_timestamp.shape', camera_timestamp.shape)
# # assert camera_timestamp[0]['id'] <= start_id
#
# ref_id = int(start_id - camera_timestamp[0]['id'])
# # select reference time stamp
# ref_time = camera_timestamp[ref_id]['desired_ts']
# print('ref_time', ref_time)
# # calculate rtol to find the correct time stamp
# # see https://numpy.org/doc/stable/reference/generated/numpy.isclose.html
# # absolute(a - b) <= desired_tol <= (atol + rtol * absolute(b))
# # use atol = 1e-5 and IMU data with 100 Hz
# freq_IMU = 100
# desired_tol = 1 / freq_IMU / 2
# # # use np.isclose()
# # atol = 1e-8
# # rtol = (desired_tol - atol) / math.fabs(ref_time)
# # if is_scientific_notation(str(rtol)):
# #     rtol = round_float_in_scientific_notation(rtol, up=True)
# #
# # ref_time_id = np.where(np.isclose(quaternion_with_timestamp[:, 1], ref_time, rtol=rtol))
#
# ref_time_id = np.where(np.abs(quaternion_with_timestamp[:, 1] - ref_time) < desired_tol)
#
# assert ref_time_id[0].size == 1
#
# ref_q = quaternion_with_timestamp[ref_time_id[0][0], 2:6]
# print(ref_q)
#
start_frame_id = 32

lla_ref = lla_list[32, 0:3]
# print(lla_ref)
enu = lla2enu(lla_list[:, 0], lla_list[:, 1], lla_list[:, 2], lla_ref[0], lla_ref[1], lla_ref[2], degrees=True)
# print(enu.shape)
# print(quaternion_with_timestamp.shape)
#
# camera_pose_path = dataset_dir / 'slam' / slam_method / experiment_id / 'frame_trajectory.txt'
# camera_traj = read_camera_trajectory(camera_pose_path)
# print(np.min(camera_traj[:, 0, 3]))
# print(np.min(camera_traj[:, 1, 3]))
# print(np.min(camera_traj[:, 2, 3]))
# print(np.max(camera_traj[:, 0, 3]))
# print(np.max(camera_traj[:, 1, 3]))
# print(np.max(camera_traj[:, 2, 3]))
#
# assert camera_timestamp[-1]['id'] == start_id + camera_traj.shape[0]
#
# # read configuration file
# config_name = 'Novatel_20231024_Stereo.txt'
# read_config(dataset_dir / config_name)

# left_camera = np.array([-458.0, 46.3, 497.3])
# antenna_1 = np.array([-525.6, 5.0, 553.5])
# Ta_lc = left_camera - antenna_1
# print(Ta_lc)
# Ta_lc = Ta_lc / 1000
#
# Ra_lc = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]])
#
# TFa_lc = np.zeros((4, 4))
# TFa_lc[0:3, 0:3] = Ra_lc
# TFa_lc[0:3, 3] = Ta_lc
# TFa_lc[3, 3] = 1
# print(TFa_lc)
#
# camera_traj = read_camera_path(camera_pose_path)
# camera_traj = camera_traj.transpose(1, 2, 0)
# camera_traj = TFa_lc @ camera_traj

# Create a 2x3 grid for subplots
gs = gridspec.GridSpec(2, 3)

ax0 = plt.subplot(gs[0, 0])
ax0.plot(enu[0, :], enu[1, :])
ax0.axis('equal')
ax0.set_xlabel('East m')
ax0.set_ylabel('North m')
ax0.set_title('East vs. North')

ax1 = plt.subplot(gs[0, 1])
ax1.plot(enu[0, :], enu[2, :])
ax1.set_xlabel('East m')
ax1.set_ylabel('Up m')
ax1.set_title('East vs. Up')

ax2 = plt.subplot(gs[0, 2])
ax2.plot(enu[1, :], enu[2, :])
ax2.set_xlabel('North m')
ax2.set_ylabel('Up m')
ax2.set_title('North vs. Up')

ax3 = plt.subplot(gs[1, :])
ax3.plot(time_stamp_list - time_stamp_list[0], lla_list[:, 3], label='std of latitude')
ax3.plot(time_stamp_list - time_stamp_list[0], lla_list[:, 4], label='std of longitude')
ax3.plot(time_stamp_list - time_stamp_list[0], lla_list[:, 5], label='std of altitude')
ax3.set_xlabel('Time s')
ax3.set_xlim(left=0)
ax3.set_ylabel('Standard deviation m')
ax3.set_title('Standard deviation of Latitude, Longitude and Altitude')
ax3.legend()

plt.tight_layout()
plt.show()

# camera_pose_path = '/home/siyuchen/lib/ORB_SLAM3/Examples/CameraTrajectory.txt'
# left_camera = np.array([-458.0, 46.3, 497.3])
# antenna_1 = np.array([-525.6, 5.0, 553.5])
# Ta_lc = left_camera - antenna_1
# print(Ta_lc)
# Ta_lc = Ta_lc / 1000
#
# Ra_lc = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]])
#
# TFa_lc = np.zeros((4, 4))
# TFa_lc[0:3, 0:3] = Ra_lc
# TFa_lc[0:3, 3] = Ta_lc
# TFa_lc[3, 3] = 1
# print(TFa_lc)
#
# camera_traj = read_camera_path(camera_pose_path)
# camera_traj = camera_traj.transpose(1, 2, 0)
# camera_traj = TFa_lc @ camera_traj


