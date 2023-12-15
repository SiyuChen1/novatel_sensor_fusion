import re
import sys
import math
from pathlib import Path
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import numpy as np
import pymap3d as pm
from scipy.spatial.transform import Rotation as R
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
    data_type = [('real_ts', np.float64), ('desired_ts', np.float64), ('id', np.int32)]
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
                # rotation from enu frame to vehicle frame
                msg = line.split(';')
                data = msg[1].split(',')
                gps_week = int(data[0])
                seconds_in_week = float(data[1])

                current_posix_time = gps_time_to_posix_time(gps_week, seconds_in_week)

                w = float(data[2])
                x = float(data[3])
                y = float(data[4])
                z = float(data[5])
                quaternion_with_timestamp.append([current_posix_time, w, x, y, z])

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


dataset_dir = Path('/home/siyuchen/Downloads/data/dataset/rivercloud_dataset/20231031/20231031_105114')
slam_method = 'orb3'
# experiment_timestamp = '231207_130517'
# experiment_timestamp = '231208_151051'
experiment_timestamp = '231207_151626'

# read start_id used in slam
start_id = None
with open(dataset_dir / 'slam' / slam_method / experiment_timestamp / 'start_id.txt', 'r') as file:
    for line in file:
        start_id = int(line)

print("start_id used in VSLAM:", start_id)


# read IMU and GPS data
imu_file_name = 'IMUData_20231031_105114.log'
lla_with_timestamp, quaternion_with_timestamp, \
    imu_with_timestamp = get_gps_imu_data(dataset_dir, imu_file_name)

lla_list = np.array([lla_msg.lla_with_std for lla_msg in lla_with_timestamp])
time_stamp_list = np.array([lla_msg.time_stamp for lla_msg in lla_with_timestamp])
quaternion_with_timestamp = np.array(quaternion_with_timestamp)

# read camera timestamp
camera_timestamp_name = 'Stereocam_20231031_105114.log'
camera_timestamp = read_camera_timestamp(dataset_dir / camera_timestamp_name)
# print('camera_timestamp.shape', camera_timestamp.shape)
assert camera_timestamp[0]['id'] <= start_id

ref_id = int(start_id - camera_timestamp[0]['id'])
# select reference time stamp
ref_time_gps_seconds = camera_timestamp[ref_id]['desired_ts']
# print("ref_time_gps_seconds:", float(ref_time_gps_seconds))
gps_week = 2286
ref_camera_posix_time = gps_time_to_posix_time(gps_week, ref_time_gps_seconds)
print('reference camera timestamp:', ref_camera_posix_time)

# calculate rtol to find the correct time stamp
# see https://numpy.org/doc/stable/reference/generated/numpy.isclose.html
# absolute(a - b) <= desired_tol <= (atol + rtol * absolute(b))
# use atol = 1e-5 and IMU data with 100 Hz
freq_IMU = 100
desired_tol = 1 / freq_IMU / 2
# # use np.isclose()
# atol = 1e-8
# rtol = (desired_tol - atol) / math.fabs(ref_time)
# if is_scientific_notation(str(rtol)):
#     rtol = round_float_in_scientific_notation(rtol, up=True)
#
# ref_time_id = np.where(np.isclose(quaternion_with_timestamp[:, 1], ref_time, rtol=rtol))

ref_lla_time_id = np.where(np.abs(time_stamp_list - ref_camera_posix_time) < desired_tol)
ref_q_time_id = np.where(np.abs(quaternion_with_timestamp[:, 0] - ref_camera_posix_time) < desired_tol)

assert ref_lla_time_id[0].size == 1
assert ref_q_time_id[0].size == 1

ref_lla_time_id = ref_lla_time_id[0][0]
ref_q_time_id = ref_q_time_id[0][0]

lla_ref = lla_list[ref_lla_time_id, 0:3]

np.set_printoptions(suppress=True)
print("reference lla timestamp:", time_stamp_list[ref_lla_time_id])
enu = lla2enu(lla_list[:, 0], lla_list[:, 1], lla_list[:, 2], lla_ref[0], lla_ref[1], lla_ref[2], degrees=True)

ref_q = quaternion_with_timestamp[ref_q_time_id, 1:5]
print("reference quaternion timestamp:", quaternion_with_timestamp[ref_q_time_id, 0])
# print(ref_q)

camera_pose_path = dataset_dir / 'slam' / slam_method / experiment_timestamp / 'frame_trajectory.txt'
camera_traj = read_camera_trajectory(camera_pose_path)
# print(np.min(camera_traj[:, 0, 3]))
# print(np.min(camera_traj[:, 1, 3]))
# print(np.min(camera_traj[:, 2, 3]))
# print(np.max(camera_traj[:, 0, 3]))
# print(np.max(camera_traj[:, 1, 3]))
# print(np.max(camera_traj[:, 2, 3]))
# print("camera_traj.shape", camera_traj.shape)

# assert camera_timestamp[-1]['id'] == start_id + camera_traj.shape[0]

tf_imu_ant1 = np.zeros((4, 4))
# ref_q with [w, x, y, z] but R.from_quat requires [x, y, z, w]
rotation_ant1_imu = R.from_quat([ref_q[1], ref_q[2], ref_q[3], ref_q[0]])
tf_imu_ant1[0:3, 0:3] = rotation_ant1_imu.as_matrix().T
# translation in mm, see email on 20.11.2023 at 15:30 from Effkemann
tf_imu_ant1[0:3, 3] = [-530.6, 65.4, 226.0]
tf_imu_ant1[0:3, 3] = 1 / 1e3 * tf_imu_ant1[0:3, 3]
# print("tf_imu_ant1:", tf_imu_ant1)

tf_ant1_imu = np.zeros((4, 4))
tf_ant1_imu[0:3, 0:3] = rotation_ant1_imu.as_matrix()
tf_ant1_imu[0:3, 3] = -rotation_ant1_imu.as_matrix() @ tf_imu_ant1[0:3, 3]

# from imu to left camera
imu_cam_left = np.zeros((3, 3))
imu_cam_left[:, 0] = [0.999258, 0.000897, 0.038500]
imu_cam_left[:, 1] = [-0.038439, -0.037827, 0.998545]
imu_cam_left[:, 2] = [0.002352, -0.999284, -0.037765]

tf_imu_cam_left = np.zeros((4, 4))
tf_imu_cam_left[0:3, 0:3] = imu_cam_left
tf_imu_cam_left[0:3, 3] = 1e-3 * np.array([-458.9, 92.6, 170.6])
tf_imu_cam_left[3, 3] = 1

tf_cam_left_vslam = np.diag([1, -1, -1, 1])

tf_ant1_cam_left = tf_ant1_imu @ tf_imu_cam_left @ tf_cam_left_vslam

camera_traj_in_ant1 = np.einsum('ij,kjl->kil', tf_ant1_cam_left, camera_traj)
camera_traj_len = camera_traj_in_ant1.shape[0]
# print(camera_traj_in_ant1.shape)

# Create a 2x3 grid for subplots
gs = gridspec.GridSpec(3, 6)

ax0 = plt.subplot(gs[0, 0:2])
ax0.plot(enu[0, :], enu[1, :], label="RTK")
ax0.plot(camera_traj_in_ant1[:, 0, 3], camera_traj_in_ant1[:, 1, 3], label="VSLAM")
ax0.axis('equal')
ax0.set_xlabel('East m')
ax0.set_ylabel('North m')
ax0.legend()
ax0.set_title('East vs. North')

ax1 = plt.subplot(gs[0, 2:4])
ax1.plot(enu[0, :], enu[2, :], label="RTK")
ax1.plot(camera_traj_in_ant1[:, 0, 3], camera_traj_in_ant1[:, 2, 3], label="VSLAM")
ax1.legend()
ax1.set_xlabel('East m')
ax1.set_ylabel('Up m')
ax1.set_title('East vs. Up')

ax2 = plt.subplot(gs[0, 4:6])
ax2.plot(enu[1, :], enu[2, :], label="RTK")
ax2.plot(camera_traj_in_ant1[:, 1, 3], camera_traj_in_ant1[:, 2, 3], label="VSLAM")
ax2.legend()
ax2.set_xlabel('North m')
ax2.set_ylabel('Up m')
ax2.set_title('North vs. Up')

ax3 = plt.subplot(gs[1, :])
ax3.plot(time_stamp_list - time_stamp_list[0], lla_list[:, 3], label='std of latitude')
ax3.plot(time_stamp_list - time_stamp_list[0], lla_list[:, 4], label='std of longitude')
ax3.plot(time_stamp_list - time_stamp_list[0], lla_list[:, 5], label='std of altitude')
ax3.set_xlabel('Time s')
ax3.set_xlim(left=0)
ax3.set_ylabel('Standard Deviation m')
ax3.set_title('Standard Deviation of Latitude, Longitude and Altitude, RTK Measurements')
ax3.legend()

camera_end_id = ref_id + camera_traj_len - 1
camera_end_time = gps_time_to_posix_time(gps_week, camera_timestamp[camera_end_id]['desired_ts'])
lla_end_id = np.where(np.abs(time_stamp_list - camera_end_time) < desired_tol)
assert lla_end_id[0].size == 1
lla_end_id = lla_end_id[0][0]

time_meas = np.array(time_stamp_list[ref_lla_time_id:lla_end_id+1] - time_stamp_list[ref_lla_time_id])

ax4 = plt.subplot(gs[2, 0])
ax4.plot(time_meas,
         enu[0, ref_lla_time_id:lla_end_id + 1], label='RTK in East')
ax4.plot(time_meas,
         camera_traj_in_ant1[:, 0, 3], label='VSLAM in East')
ax4.set_xlabel('Time s')
ax4.set_xlim(left=0)
ax4.set_ylabel('Meter m')
ax4.legend()
ax4.set_title('RTK vs. VSLAM in East')

ax5 = plt.subplot(gs[2, 1])
dif_east = np.array(camera_traj_in_ant1[:, 0, 3]) - np.array(enu[0, ref_lla_time_id:lla_end_id + 1])
ax5.plot(time_meas, np.abs(dif_east), 'g--', label="difference")
ax5.legend()

ax6 = plt.subplot(gs[2, 2])
ax6.plot(time_meas,
         enu[1, ref_lla_time_id:lla_end_id + 1], label='RTK in North')
ax6.plot(time_meas,
         camera_traj_in_ant1[:, 1, 3], label='VSLAM in North')
ax6.set_xlabel('Time s')
ax6.set_xlim(left=0)
ax6.set_ylabel('Meter m')
ax6.legend()
ax6.set_title('RTK vs. VSLAM in North')

ax7 = plt.subplot(gs[2, 3])
dif_north = np.array(camera_traj_in_ant1[:, 1, 3]) - np.array(enu[1, ref_lla_time_id:lla_end_id + 1])
ax7.plot(time_meas, np.abs(dif_north), 'g--', label="difference")
ax7.legend()

ax8 = plt.subplot(gs[2, 4])
ax8.plot(time_meas,
         enu[2, ref_lla_time_id:lla_end_id + 1], label='RTK in Up')
ax8.plot(time_meas,
         camera_traj_in_ant1[:, 2, 3], label='VSLAM in Up')
ax8.set_xlabel('Time s')
ax8.set_xlim(left=0)
ax8.set_ylabel('Meter m')
ax8.legend()
ax8.set_title('RTK vs. VSLAM in Up')

ax9 = plt.subplot(gs[2, 5])
dif_up = np.array(camera_traj_in_ant1[:, 2, 3]) - np.array(enu[2, ref_lla_time_id:lla_end_id + 1])
ax9.plot(time_meas, np.abs(dif_up), 'g--', label="difference")
ax9.legend()

# # plt.tight_layout()
# plt.subplot_tool()

# set the spacing between subplots
plt.subplots_adjust(left=0.1,
                    bottom=0.1,
                    right=0.9,
                    top=0.9,
                    wspace=0.4,
                    hspace=0.4)

plt.show()

# # read configuration file
# config_name = 'Novatel_20231024_Stereo.txt'
# read_config(dataset_dir / config_name)
#
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
#
#
#
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

# imu_with_timestamp = np.array(
#     [[imu_msg.time_stamp, imu_msg.angular_velocity.x,
#       imu_msg.angular_velocity.y, imu_msg.angular_velocity.z,
#       imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y,
#       imu_msg.linear_acceleration.z] for imu_msg in imu_with_timestamp])
# print(imu_with_timestamp.shape)
#
# imu_header = '''#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],
# a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]'''
#
# imu_header = imu_header.replace('\n', '').strip()
#
# # Export array with header
# # comments='' to avoid the default comment character (#) being added before your header.
# np.savetxt(dataset_dir / 'imu_ts.txt', imu_with_timestamp,
#            delimiter=',', header=imu_header, comments='')
