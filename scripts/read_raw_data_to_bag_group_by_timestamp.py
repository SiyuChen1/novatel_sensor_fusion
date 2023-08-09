#!/usr/bin/env python3

from geometry_msgs.msg import TransformStamped
from novatel_sensor_fusion.msg import NavRMC, NavSat, NavSatExtended
from novatel_sensor_fusion_py.raw_data_to_bag.data2rosmsg import gps_time_to_ros_time
from novatel_sensor_fusion_py.ultilies.datatime_convert import dms_to_decimal
from novatel_sensor_fusion_py.ultilies.group_raw_data_by_timestamp import read_data_into_dataframe, parse_line
import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from rclpy.time import Time
import rosbag2_py
from sensor_msgs.msg import Imu
from tf2_msgs.msg import TFMessage
import transforms3d


class ImuRawDataBagRecorder(Node):
    def __init__(self, topic_infos, raw_data_file_path, bag_name, imu_data_rate, translation):
        super().__init__('imu_raw_data_bag_recorder')
        self.writer = rosbag2_py.SequentialWriter()

        storage_options = rosbag2_py._storage.StorageOptions(
            uri=bag_name,
            storage_id='mcap')
        converter_options = rosbag2_py._storage.ConverterOptions('', '')
        self.writer.open(storage_options, converter_options)

        for topic_name in topic_infos.keys():
            topic_type = topic_infos[topic_name]
            topic_info = rosbag2_py._storage.TopicMetadata(
                name=topic_name,
                type=topic_type,
                serialization_format='cdr')
            self.writer.create_topic(topic_info)

        # additionally publish /tf topic
        topic_info = rosbag2_py._storage.TopicMetadata(
            name='/tf',
            type='tf2_msgs/msg/TFMessage',
            serialization_format='cdr')
        self.writer.create_topic(topic_info)

        self.imu_data_rate = imu_data_rate
        # the translation from instantaneous local level frame (ENU) to vehicle frame
        self.translation = translation

        # Reading the data into a DataFrame
        imu_data_df = read_data_into_dataframe(raw_data_file_path)

        # Applying the parse_line function to the DataFrame
        parsed_data_df = imu_data_df.apply(parse_line, axis=1).dropna()

        # Grouping the data by GPS time
        self.grouped_data_df = list(parsed_data_df.groupby('gps_time'))

    def write_data_to_bag(self):
        only_best = 0
        only_bestvel = 0
        only_bestgnss = 0
        only_bestgnssvel = 0
        for indx in range(len(self.grouped_data_df)):
            data = self.grouped_data_df[indx]
            gps_time_str = data[0].split(',')
            # print(gps_time_str)

            # print(type(data[1]))
            # <class 'pandas.core.frame.DataFrame'>

            # print(data[1].columns)
            # Index(['command_type', 'gps_time', 'raw_line'], dtype='object')

            stamp = gps_time_to_ros_time(gps_week=gps_time_str[0], seconds_in_week=gps_time_str[1])
            # print(stamp)
            # len(data[1]) is the number of raw data lines

            command_types = list(data[1]['command_type'])
            if 'BESTPOSA' in command_types or 'BESTVELA' in command_types:
                bestpos = NavSatExtended()
                bestpos.header.stamp = stamp
                bestpos.header.frame_id = 'antenna1'

            if 'BESTGNSSPOSA' in command_types or 'BESTGNSSVELA' in command_types:
                bestgnsspos = NavSatExtended()
                bestgnsspos.header.stamp = stamp
                bestgnsspos.header.frame_id = 'antenna1'

            if 'CORRIMUSA' in command_types or 'INSATTQSA' in command_types:
                imu_msg = Imu()
                imu_msg.header.stamp = stamp
                imu_msg.header.frame_id = 'vehicle'

            for _, row in data[1].iterrows():
                # print(row['command_type'])
                # print(row['raw_line'])
                raw_data = row['raw_line']
                if ';' in raw_data:
                    data = raw_data.split(';')[1]
                else:
                    data = raw_data
                data = data.split(',')
                command_type = row['command_type']

                match command_type:
                    case 'CORRIMUSA':
                        imu_data_count = int(data[0])
                        if imu_data_count > 0:
                            imu_msg.angular_velocity.x = self.imu_data_rate / imu_data_count * float(data[1])
                            imu_msg.angular_velocity.y = self.imu_data_rate / imu_data_count * float(data[2])
                            imu_msg.angular_velocity.z = self.imu_data_rate / imu_data_count * float(data[3])
                            imu_msg.linear_acceleration.x = self.imu_data_rate / imu_data_count * float(data[4])
                            imu_msg.linear_acceleration.y = self.imu_data_rate / imu_data_count * float(data[5])
                            imu_msg.linear_acceleration.z = self.imu_data_rate / imu_data_count * float(data[6])
                    case 'INSATTQSA':
                        w = float(data[2])
                        x = float(data[3])
                        y = float(data[4])
                        z = float(data[5])
                        imu_msg.orientation.w = w
                        imu_msg.orientation.x = x
                        imu_msg.orientation.y = y
                        imu_msg.orientation.z = z

                        msg = TransformStamped()
                        # i means instantaneous
                        msg.header.stamp = stamp
                        msg.header.frame_id = 'antenna1'
                        msg.child_frame_id = 'vehicle'

                        r = transforms3d.quaternions.quat2mat([w, x, y, z])
                        translation_base_to_child = - r @ self.translation

                        msg.transform.rotation = imu_msg.orientation

                        msg.transform.translation.x = translation_base_to_child[0]
                        msg.transform.translation.z = translation_base_to_child[1]
                        msg.transform.translation.y = translation_base_to_child[2]

                        tf_msg = TFMessage()
                        tf_msg.transforms = [msg]
                        self.writer.write(
                            '/tf', serialize_message(tf_msg),
                            Time.from_msg(stamp).nanoseconds)

                    case 'INSATTXA':
                        # this log contains redundant rotation information from local level frame to vehicle body frame
                        # which is redundant to the log 'INSATTQSA', additionally this log contains the standard
                        # deviation of the roll, pitch, yaw angles
                        pass

                    case 'GPRMC':
                        gprmc_msg = NavRMC()
                        gprmc_msg.header.stamp = stamp
                        gprmc_msg.header.frame_id = 'antenna1'
                        lat_str = data[3]
                        lon_str = data[5]
                        gprmc_msg.latitude = dms_to_decimal(
                            degrees=int(lat_str[0:2]),
                            minutes=float(lat_str[2:-1])
                        )
                        gprmc_msg.longitude = dms_to_decimal(
                            degrees=int(lon_str[0:3]),
                            minutes=float(lon_str[3:-1])
                        )
                        if data[2] == 'A':
                            gprmc_msg.is_pos_valid = True
                        else:
                            gprmc_msg.is_pos_valid = False

                        gprmc_msg.ground_speed = float(data[7])
                        gprmc_msg.track_angle = float(data[8])
                        gprmc_msg.date = str(data[9])

                        self.writer.write(
                            '/gprmc', serialize_message(gprmc_msg),
                            Time.from_msg(stamp).nanoseconds)

                    case 'GPGGA':
                        msg = NavSat()
                        msg.header.stamp = stamp
                        msg.header.frame_id = 'antenna1'

                        lat_str = data[2]
                        lon_str = data[4]

                        msg.latitude = dms_to_decimal(
                            degrees=int(lat_str[0:2]),
                            minutes=float(lat_str[2:-1])
                        )

                        msg.longitude = dms_to_decimal(
                            degrees=int(lon_str[0:3]),
                            minutes=float(lon_str[3:-1])
                        )

                        msg.altitude = float(data[9])

                        msg.quality = str(data[6])
                        msg.nb_sat_in_use = int(data[7])
                        msg.undulation = float(data[11])

                        if len(data[2]) > 9:
                            # in this case, log command is GPGGALONG
                            topic_name = '/gpggalong'
                        else:
                            topic_name = '/gpgga'
                        self.writer.write(
                            topic_name, serialize_message(msg),
                            Time.from_msg(stamp).nanoseconds)

                    case 'BESTPOSA':
                        bestpos.pos_sol_status = str(data[0])
                        bestpos.pos_type = str(data[1])
                        bestpos.latitude = float(data[2])
                        bestpos.longitude = float(data[3])
                        bestpos.altitude = float(data[4])
                        bestpos.undulation = float(data[5])
                        bestpos.position_covariance[0] = float(data[7])
                        bestpos.position_covariance[4] = float(data[8])
                        bestpos.position_covariance[8] = float(data[9])
                        bestpos.nb_sat_tracked = int(data[13])
                        bestpos.nb_sat_solution = int(data[14])

                    case 'BESTVELA':
                        bestpos.vel_sol_status = str(data[0])
                        bestpos.vel_type = str(data[1])
                        bestpos.horizontal_speed = float(data[4])
                        bestpos.direction_to_north = float(data[5])
                        bestpos.vertical_speed = float(data[6])

                    case 'BESTGNSSPOSA':
                        bestgnsspos.pos_sol_status = str(data[0])
                        bestgnsspos.pos_type = str(data[1])
                        bestgnsspos.latitude = float(data[2])
                        bestgnsspos.longitude = float(data[3])
                        bestgnsspos.altitude = float(data[4])
                        bestgnsspos.undulation = float(data[5])
                        bestgnsspos.position_covariance[0] = float(data[7])
                        bestgnsspos.position_covariance[4] = float(data[8])
                        bestgnsspos.position_covariance[8] = float(data[9])
                        bestgnsspos.nb_sat_tracked = int(data[13])
                        bestgnsspos.nb_sat_solution = int(data[14])

                    case 'BESTGNSSVELA':
                        bestgnsspos.vel_sol_status = str(data[0])
                        bestgnsspos.vel_type = str(data[1])
                        bestgnsspos.horizontal_speed = float(data[4])
                        bestgnsspos.direction_to_north = float(data[5])
                        bestgnsspos.vertical_speed = float(data[6])

            if 'BESTPOSA' in command_types and 'BESTVELA' in command_types:
                self.writer.write(
                    '/best', serialize_message(bestpos),
                    Time.from_msg(stamp).nanoseconds)
            elif 'BESTPOSA' in command_types:
                only_best += 1
            elif 'BESTVELA' in command_types:
                only_bestvel += 1

            if 'BESTGNSSPOSA' in command_types and 'BESTGNSSVELA' in command_types:
                self.writer.write(
                    '/bestgnss', serialize_message(bestgnsspos),
                    Time.from_msg(stamp).nanoseconds)
            elif 'BESTGNSSPOSA' in command_types:
                only_bestgnss += 1
            elif 'BESTGNSSVELA' in command_types:
                only_bestgnssvel += 1

            if 'CORRIMUSA' in command_types or 'INSATTQSA' in command_types:
                self.writer.write(
                    '/imu', serialize_message(imu_msg),
                    Time.from_msg(stamp).nanoseconds)

        print(f'In msg, {only_bestgnss} only has bestgnsspos')
        print(f'In msg, {only_bestgnssvel} only has bestgnssposvel')
        print(f'In msg, {only_best} only has bestpos')
        print(f'In msg, {only_bestvel} only has bestvel')


def main(args=None):
    rclpy.init(args=args)
    topic_infos = {'/bestgnss': 'novatel_sensor_fusion/msg/NavSatExtended',
                   '/best': 'novatel_sensor_fusion/msg/NavSatExtended',
                   '/gpgga': 'novatel_sensor_fusion/msg/NavSat',
                   '/gpggalong': 'novatel_sensor_fusion/msg/NavSat',
                   '/gprmc': 'novatel_sensor_fusion/msg/NavRMC',
                   '/imu': 'sensor_msgs/msg/Imu'}

    file_path = '/home/siyuchen/Documents/Novatel_Stereocam_Daten_20230703' \
                '/20230703_140222/IMUData_20230703_140222.log'

    # file_path = '/home/siyuchen/catkin_ws/src/novatel_sensor_fusion' \
    #             '/jupyter_notebook/data/small_raw_imu_data.txt'

    imu_data_rate = 100

    rosbag_name = 'imu_raw_data_bag_recorder'

    # translation_imu2ll = [dx, dy, dz] is the translation from IMU frame to
    # instantaneous local level frame (ENU), i.e. antenna1. Since after rotating 180 degree around 180 degrees
    # the IMU frame is transformed to the vehicle frame.
    # the translation from vehicle frame to the instantaneous local level frame (ENU) is [dx, -dy, -dz]
    dx = 0.431
    dy = 0.0
    dz = 0.013
    translation_imu2antenna_1 = [dx, dy, dz]
    translation = [dx, -dy, -dz]

    recorder = ImuRawDataBagRecorder(
        topic_infos=topic_infos,
        raw_data_file_path=file_path,
        bag_name=rosbag_name,
        imu_data_rate=imu_data_rate,
        translation=translation
    )

    recorder.write_data_to_bag()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
