import argparse
import matplotlib.pyplot as plt
import numpy as np


start_time = None

# https://docs.novatel.com/OEM7/Content/PDFs/OEM7_Commands_Logs_Manual.pdf
# table 278 on page 1199 for cpt7 imu
gyroscope_factor = 1 / 2 ** 33
accelerometer_factor = 1 / 2 ** 29

# https://docs.novatel.com/OEM7/Content/PDFs/OEM7_Commands_Logs_Manual.pdf
# on page 1201 and on page 1167
# for cpt 7 imu, 100 Hz
freq = 100

parser = argparse.ArgumentParser(description="Plot raw IMU data")
parser.add_argument("--input-raw-imu", type=str, default=None)

# Parse the arguments
args = parser.parse_args()

raw_imu_path = args.input_raw_imu
# Create a new bag file (replace 'test.bag' with your desired file)

raw_imu_ts_array = []
correct_imu_ts_array = []

correct_imu_count = 0
raw_imu_count = 0

with open(raw_imu_path, 'r') as file:
    for line in file:
        if line.strip() and 'RAWIMUSA' in line:
            # This log is output in the IMU Body frame. on page 1167
            # Splitting the line by spaces and converting the parts to floats
            parts = line.split(';')
            data = parts[1].rstrip().split(',')
            if int(data[0]):
                if start_time is None:
                    start_time = float(data[1])

                raw_imu_count = raw_imu_count + 1

                current_time = float(data[1])
                imu_ts = []
                imu_ts.append(100 * (current_time - start_time))
                # acceleration with order Z, Y, X
                imu_ts.append(accelerometer_factor * float(data[3]) * freq)
                imu_ts.append(-accelerometer_factor * float(data[4]) * freq)
                imu_ts.append(accelerometer_factor * float(data[5]) * freq)
                # acceleration with order Z, Y, X
                imu_ts.append(gyroscope_factor * float(data[6]) * freq)
                imu_ts.append(-gyroscope_factor * float(data[7]) * freq)
                imu_ts.append(gyroscope_factor * float(data[8].split('*')[0]) * freq)

                raw_imu_ts_array.append(imu_ts)

            if raw_imu_count % 20000 == 0:
                print(f"{raw_imu_count} raw imu data have been processed")
        elif line.strip() and 'CORRIMUSA' in line:
            # This log is not output in the IMU Body frame, but in the vehicle body frame. on page 1100
            # Set by SETINSROTATION RBV 180 0 0. See Novatel_20231201_IMU_Calibration.txt
            # Splitting the line by spaces and converting the parts to floats
            parts = line.split(';')
            data = parts[1].rstrip().split(',')
            header = parts[0].rstrip().split(',')

            imu_data_count = int(data[0])

            if imu_data_count > 0:
                if start_time is None:
                    start_time = float(header[2])

                current_time = float(header[2])

                correct_imu_count = correct_imu_count + 1
                imu_ts = []
                imu_ts.append(100 * (current_time - start_time))

                # acceleration
                # firstly in Z, factor -1 because of rotation between IMU body and vehicle frame
                imu_ts.append(- 1 / imu_data_count * float(data[6]) * freq)
                # then in Y, factor -1 because of rotation between IMU body and vehicle frame
                imu_ts.append(- 1 / imu_data_count * float(data[5]) * freq)
                # at last in X
                imu_ts.append(1 / imu_data_count * float(data[4]) * freq)

                # angular velocity
                # firstly in Z, factor -1 because of rotation between IMU body and vehicle frame
                imu_ts.append(- 1 / imu_data_count * float(data[3]) * freq)
                # then in Y, factor -1 because of rotation between IMU body and vehicle frame
                imu_ts.append(- 1 / imu_data_count * float(data[2]) * freq)
                # at last in X
                imu_ts.append(1 / imu_data_count * float(data[1]) * freq)

                correct_imu_ts_array.append(imu_ts)

            if correct_imu_count % 20000 == 0:
                print(f"{correct_imu_count} correct imu data have been processed")

print("correct_imu_count/raw_imu_count", correct_imu_count, "/", raw_imu_count)

imu_ts_array_np = np.array(raw_imu_ts_array)
cor_imu_ts_array_np = np.array(correct_imu_ts_array)

print("before unique: imu_ts_array_np.shape = ", imu_ts_array_np.shape)
# Find unique timestamps
_, unique_indices = np.unique(imu_ts_array_np[:, 0].astype(int), return_index=True)
imu_ts_array_np = imu_ts_array_np[unique_indices]
print("after unique: imu_ts_array_np.shape = ", imu_ts_array_np.shape)

print("before unique: cor_imu_ts_array_np.shape = ", cor_imu_ts_array_np.shape)
_, unique_indices = np.unique(cor_imu_ts_array_np[:, 0].astype(int), return_index=True)
cor_imu_ts_array_np = cor_imu_ts_array_np[unique_indices]
print("after unique: cor_imu_ts_array_np.shape = ", cor_imu_ts_array_np.shape)

common_timestamps = np.intersect1d(
    imu_ts_array_np[:, 0].astype(int),
    cor_imu_ts_array_np[:, 0].astype(int))

# Filter rows based on common timestamps
filtered_raw_imu = imu_ts_array_np[
    np.isin(imu_ts_array_np[:, 0].astype(int), common_timestamps)]
filtered_cor_imu = cor_imu_ts_array_np[
    np.isin(cor_imu_ts_array_np[:, 0].astype(int), common_timestamps)]

print(filtered_cor_imu.shape)
print(filtered_raw_imu.shape)

print(filtered_raw_imu[-1], filtered_cor_imu[-1])

g = filtered_cor_imu[:, 1:4] + filtered_raw_imu[:, 1:4]
t_diff = filtered_cor_imu[:, 0] - filtered_raw_imu[:, 0]

# Creating the figure
plt.figure(figsize=(15, 10))

# First Row: Acceleration Plots
plt.subplot(2, 3, 1)  # 2 rows, 3 columns, 1st subplot
plt.plot(filtered_raw_imu[:, 3], color='r')
plt.plot(filtered_cor_imu[:, 3], color='g')
plt.title('Acceleration in X')
plt.xlabel('Time (s)')
plt.ylabel('Acc X')

plt.subplot(2, 3, 2)  # 2 rows, 3 columns, 2nd subplot
plt.plot(filtered_raw_imu[:, 2], color='r')
plt.plot(filtered_cor_imu[:, 2], color='g')
plt.title('Acceleration in Y')
plt.xlabel('Time (s)')
plt.ylabel('Acc Y')

plt.subplot(2, 3, 3)  # 2 rows, 3 columns, 3rd subplot
plt.plot(filtered_raw_imu[:, 1], color='r')
plt.plot(filtered_cor_imu[:, 1], color='g')
plt.title('Acceleration in Z')
plt.xlabel('Time (s)')
plt.ylabel('Acc Z')

plt.subplot(2, 1, 2)  # 2 rows, 3 columns, 3rd subplot
plt.plot(1 / 100 * filtered_cor_imu[:, 0], np.linalg.norm(g, axis=1))
plt.xlabel('Time (s)')
plt.ylabel('Acceleration (m/s^2)')
# plt.plot(t_diff)

# Display the plots
plt.tight_layout()
plt.show()
# # Extracting individual columns
# time_stamps = imu_ts_array_np[:, 0]
# acc_x = imu_ts_array_np[:, 1]
# acc_y = imu_ts_array_np[:, 2]
# acc_z = imu_ts_array_np[:, 3]
# ang_vel_x = imu_ts_array_np[:, 4]
# ang_vel_y = imu_ts_array_np[:, 5]
# ang_vel_z = imu_ts_array_np[:, 6]
#
# # Creating the figure
# plt.figure(figsize=(15, 10))
#
# # First Row: Acceleration Plots
# plt.subplot(2, 3, 1)  # 2 rows, 3 columns, 1st subplot
# plt.plot(time_stamps, acc_x, color='r')
# plt.title('Acceleration in X')
# plt.xlabel('Time (s)')
# plt.ylabel('Acc X')
#
# plt.subplot(2, 3, 2)  # 2 rows, 3 columns, 2nd subplot
# plt.plot(time_stamps, acc_y, color='g')
# plt.title('Acceleration in Y')
# plt.xlabel('Time (s)')
# plt.ylabel('Acc Y')
#
# plt.subplot(2, 3, 3)  # 2 rows, 3 columns, 3rd subplot
# plt.plot(time_stamps, acc_z, color='b')
# plt.title('Acceleration in Z')
# plt.xlabel('Time (s)')
# plt.ylabel('Acc Z')
#
# # Second Row: Angular Velocity Plot
# plt.subplot(2, 1, 2)  # 2 rows, 1 column, 2nd subplot
# plt.plot(time_stamps, ang_vel_x, label='Ang Vel X', color='r')
# plt.plot(time_stamps, ang_vel_y, label='Ang Vel Y', color='g')
# plt.plot(time_stamps, ang_vel_z, label='Ang Vel Z', color='b')
# plt.title('IMU Angular Velocity Data')
# plt.xlabel('Time (s)')
# plt.ylabel('Angular Velocity')
# plt.legend()
#
# # Display the plots
# plt.tight_layout()
# plt.show()
