import argparse
from gps_time import GPSTime
import matplotlib.pyplot as plt
from novatel_sensor_fusion_py.ultilies.quaternion_multiply import quaternion_multiply
import numpy as np
import os
from pathlib import Path


abs_path = Path(os.path.abspath(__file__))
parser = argparse.ArgumentParser(description="Check gyroscope measurement with ground truth")
parser.add_argument("--folder-path", type=str,
                    default=str(abs_path.parent.parent.parent / 'data/gnss_imu_logs/determine_gnss_pos'))
parser.add_argument("--file-name", type=str,
                    default='Rotation um Antenne1_2023-07-03_14-36-08.log')

# Parse the arguments
args = parser.parse_args()
folder_path = args.folder_path
file_name = args.file_name

imu_data_rate = 100

ref_gps_week = 2269
ref_gps_after_week_sec = 131786.240000000
end_gps_week = 2269
end_gps_after_week_sec = 131801.340000000

ref_time = GPSTime(week_number=ref_gps_week, time_of_week=ref_gps_after_week_sec)
end_time = GPSTime(week_number=end_gps_week, time_of_week=end_gps_after_week_sec)

nb = int(imu_data_rate * (end_time - ref_time)) + 1
print('theoretical number of measurements:', nb)

q_list = np.zeros((nb, 4))
gyro_list = np.zeros((nb, 3))
q_list.fill(np.nan)
gyro_list.fill(np.nan)

with open(Path(folder_path) / file_name) as data_file:
    content = data_file.read().split('\n')
    for line in content:
        if 'CORRIMUSA' in line:
            msg = line.split(';')
            header = msg[0].split(',')
            data = msg[1].split(',')
            imu_data_count = float(data[0])
            if imu_data_count > 0:
                avx = float(data[1]) * imu_data_rate / imu_data_count
                avy = float(data[2]) * imu_data_rate / imu_data_count
                avz = float(data[3]) * imu_data_rate / imu_data_count
                cur_time = GPSTime(week_number=int(header[1]), time_of_week=float(header[2]))
                gyro_id = int(imu_data_rate * (cur_time - ref_time))
                gyro_list[gyro_id, :] = [avx, avy, avz]
            else:
                print(imu_data_count)
        if 'INSATTQSA' in line:
            msg = line.split(';')
            data = msg[1].split(',')
            if 'INS_SOLUTION_GOOD' in data[-1]:
                w = float(data[2])
                x = float(data[3])
                y = float(data[4])
                z = float(data[5])
                cur_time = GPSTime(week_number=int(data[0]), time_of_week=float(data[1]))
                q_id = int(imu_data_rate * (cur_time - ref_time))
                q = [w, x, y, z]
                q_list[q_id, :] = q

print(np.nanmean(np.abs(gyro_list[:, 0] - gyro_list[:, 1])))
print('Non-nan values of quaternion: ', np.count_nonzero(~np.isnan(q_list[:, 0])))
print('Non-nan values of gyroscope measurements: ', np.count_nonzero(~np.isnan(gyro_list[:, 0])))

q_init = q_list[0, :]
q_estimated = np.zeros(q_list.shape)
q_estimated.fill(np.nan)
q_estimated[0, :] = q_init

prev_gyro_id = 0

gap_list = np.zeros(nb)
gap_list.fill(np.nan)

for index in range(1, nb):
    if not np.isnan(gyro_list[index, 0]):
        gap_list[index] = index - prev_gyro_id

        if gap_list[index] == 0:
            print('Unwanted index')

        # reference:
        # https://stackoverflow.com/questions/39441900/how-use-raw-gryoscope-data-s-for-calculating-3d-rotation

        # # method 1: using rotation matrix
        # # assuming that rotation is extrinsic with z->x->y
        # dt = (index - prev_gyro_id) / imu_data_rate
        # integrated_angles = 1 / 2 * dt * (gyro_list[prev_gyro_id, :] + gyro_list[index, :])
        # # rotation order firstly z, then x, at last y
        # Mat_incremental = transforms3d.euler.euler2mat(
        #     integrated_angles[1], integrated_angles[0], integrated_angles[2], 'syxz')
        #
        # Mat = transforms3d.quaternions.quat2mat(q_estimated[prev_gyro_id, :])
        # Mat_estimated = Mat_incremental @ Mat
        # # Quaternion in w, x, y z (real, then vector) format
        # q_estimated[index, :] = transforms3d.quaternions.mat2quat(Mat_estimated)
        # prev_gyro_id = index

        # method 2: using quaternion first order approximation
        # https://github.com/priseborough/InertialNav/blob/master/code/estimator_21states.cpp
        dt = (index - prev_gyro_id) / imu_data_rate
        integrated_angles = dt * gyro_list[index, :]
        q_delta = np.array([1, 1 / 2 * integrated_angles[0],
                            1 / 2 * integrated_angles[1], 1 / 2 * integrated_angles[2]])
        q_delta = 1 / np.linalg.norm(q_delta) * q_delta
        q_estimated[index, :] = quaternion_multiply(q_delta, q_estimated[prev_gyro_id, :])
        prev_gyro_id = index

        # # method 3: implementation in matlab
        # dt = (index - prev_gyro_id) / imu_data_rate
        # integrated_angles = 1 / 2 * dt * (gyro_list[prev_gyro_id, :] + gyro_list[index, :])
        # theta = np.linalg.norm(integrated_angles)
        # q_delta = np.zeros(4)
        # q_delta[0] = np.cos(theta / 2)
        # q_delta[1:4] = np.sin(theta / 2) / theta * integrated_angles
        # q_estimated[index, :] = quaternion_multiply(q_delta, q_estimated[prev_gyro_id, :])
        # prev_gyro_id = index

q_error = np.zeros(nb)
# q_error.fill(np.nan)

q_error2 = np.zeros(nb)
# q_error2.fill(np.nan)

for index in range(nb):
    if not np.isnan(q_list[index, 0]) and not np.isnan(q_estimated[index, 0]):
        # method 1: Norm of the Difference of Quaternions
        # https://www.cs.cmu.edu/~cga/dynopt/readings/Rmetric.pdf page 4, section 3.3
        # and equation 31
        error_1 = np.sqrt(np.sum(np.square(q_list[index, :] - q_estimated[index, :])))
        error_2 = np.sqrt(np.sum(np.square(q_list[index, :] + q_estimated[index, :])))
        error = np.min([error_1, error_2])
        # using equation 31
        error = np.arccos(1 - error ** 2 / 2)
        q_error[index] = error * 180 / np.pi

        # # method 2: Inner Product of Unit Quaternions
        # # https://www.cs.cmu.edu/~cga/dynopt/readings/Rmetric.pdf page 4, section 3.4
        q1 = q_list[index, :]
        q2 = q_estimated[index, :]
        tmp = np.abs(np.dot(q1, q2))
        if tmp > 1:
            if np.allclose(tmp, 1):
                tmp = 1.0
            else:
                print('invalid value is encountered')
        q_error2[index] = np.arccos(tmp) * 180 / np.pi

print('Averaged estimated error in degree: ', np.nanmean(q_error))
print('Non-nan values of estimation: ', np.count_nonzero(~np.isnan(q_error)))
print(np.allclose(q_error, q_error2))
print(np.max(np.abs(q_error - q_error2)))

fig, axs = plt.subplots(2, 1)
axs[0].plot(q_error, label='Norm of the Difference of Quaternions')
axs[0].plot(q_error2, label='Inner Product of Unit Quaternions')
axs[0].set_title('Rotation estimation using Gyroscope Measurements vs. Sensor Output')
axs[0].set_xlabel('Measurement count')
axs[0].set_ylabel('Error in degree')
axs[0].legend()

axs[1].plot(gap_list, '*', label='IMU update gap')
axs[1].legend()
plt.show()
