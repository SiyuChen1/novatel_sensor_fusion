import numpy as np
from pathlib import Path


def convert_imu_to_orbslam3(abs_path, file_name):
    imu_with_timestamp = []
    imu_data_rate = 100
    with open(abs_path / file_name) as data_file:
        content = data_file.read().split('\n')
        for index, line in enumerate(content):
            if 'CORRIMUSA' in line:
                msg = line.split(';')
                header = msg[0].split(',')
                data = msg[1].split(',')
                gps_week = int(header[1])
                seconds_in_week = float(header[2])
                imu_data_count = int(data[0])
                if imu_data_count > 0:
                    ts = seconds_in_week

                    imu_msg = [ts, imu_data_rate / imu_data_count * float(data[1]),
                               imu_data_rate / imu_data_count * float(data[2]),
                               imu_data_rate / imu_data_count * float(data[3]),
                               imu_data_rate / imu_data_count * float(data[4]),
                               imu_data_rate / imu_data_count * float(data[5]),
                               imu_data_rate / imu_data_count * float(data[6])]
                    imu_with_timestamp.append(imu_msg)

        imu_with_timestamp = np.array(imu_with_timestamp)

        imu_header = '''#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],
        a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]'''

        imu_header = imu_header.replace('\n', '').strip()

        # Export array with header
        # comments='' to avoid the default comment character (#) being added before your header.
        np.savetxt(abs_path / 'imu_ts.txt', imu_with_timestamp,
                   delimiter=',', header=imu_header, comments='')


dataset_dir = Path('/home/siyuchen/Documents/rivercloud_dataset/20231031/20231031_105114')
# read IMU data
imu_file_name = 'IMUData_20231031_105114.log'
convert_imu_to_orbslam3(dataset_dir, imu_file_name)
