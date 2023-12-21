#!/usr/bin/env python3

from geometry_msgs.msg import TransformStamped
from novatel_sensor_fusion.msg import NavECEF, NavRMC, NavSat, NavSatExtended, NavUTM
from novatel_sensor_fusion_py.raw_data_to_bag.data2rosmsg import gps_time_to_ros_time
from novatel_sensor_fusion_py.ultilies.datatime_convert import dms_to_decimal
from novatel_sensor_fusion_py.ultilies.group_raw_data_by_timestamp import parse_line, read_data_into_dataframe
import numpy as np
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.serialization import serialize_message
from rclpy.time import Time
import rosbag2_py
from sensor_msgs.msg import Imu
from tf2_msgs.msg import TFMessage
from scipy.spatial.transform import Rotation as R


class ImuRawDataBagRecorder(Node):
    def __init__(self, topic_infos,
                 imu_data_rate,
                 ignore=True):
        super().__init__('imu_raw_data_bag_recorder')
        """
        ignore: whether ignore the first 10% of the recorded data
        """
        self.writer = rosbag2_py.SequentialWriter()

        self.declare_parameter(
            'file_path', descriptor=ParameterDescriptor(
                name='file_path', type=4
                # https://docs.ros2.org/foxy/api/rcl_interfaces/msg/ParameterType.html
                # uint8 PARAMETER_STRING=4
            )
        )

        self.declare_parameter(
            'ros2bag_name', descriptor=ParameterDescriptor(
                name='ros2bag_name', type=4
                # https://docs.ros2.org/foxy/api/rcl_interfaces/msg/ParameterType.html
                # uint8 PARAMETER_STRING=4
            )
        )

        self.declare_parameter(
            'translation_imu_antenna1', descriptor=ParameterDescriptor(
                name='translation_imu_antenna1', type=8
                # https://docs.ros2.org/foxy/api/rcl_interfaces/msg/ParameterType.html
                # uint8 PARAMETER_STRING=4
            )
        )

        self.declare_parameter(
            'rotation_imu_vehicle', descriptor=ParameterDescriptor(
                name='ros2bag_name', type=8
                # https://docs.ros2.org/foxy/api/rcl_interfaces/msg/ParameterType.html
                # uint8 PARAMETER_STRING=4
            )
        )

        raw_data_file_path = self.get_parameter('file_path').get_parameter_value().string_value
        ros2bag_name = self.get_parameter('ros2bag_name').get_parameter_value().string_value
        translation_imu_antenna1 = \
            self.get_parameter('translation_imu_antenna1').get_parameter_value().double_array_value.tolist()
        rotation_imu_vehicle_angles = \
            self.get_parameter('rotation_imu_vehicle').get_parameter_value().double_array_value.tolist()

        self.get_logger().info(str(type(translation_imu_antenna1)))
        self.get_logger().info(str(type(rotation_imu_vehicle_angles)))

        # raw_data_file_path = '/home/siyuchen/Downloads/data/dataset/rivercloud_dataset/20231031/20231031_105114/IMUData_20231031_105114.log'
        # ros2bag_name = '20231031_105114_Blausteinsee'

        raw_data_file_path = self.get_parameter('file_path').get_parameter_value().string_value
        ros2bag_name = self.get_parameter('ros2bag_name').get_parameter_value().string_value

        storage_options = rosbag2_py._storage.StorageOptions(
            uri=ros2bag_name,
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

        # the translation from vehicle body frame (if it is set, then it is the default output frame)
        # to the antenna1
        # P(imu->antenna1) = T(imu->vehicle) + R(imu->vehicle) @ P(vehicle->antenna1)
        # thus, P(vehicle->antenna1) = inv(R(imu->vehicle)) @ (P(imu->antenna1) - T(imu->vehicle))
        # Since T(imu->vehicle) = [0, 0, 0]
        # P(vehicle->antenna1) = inv(R(imu->vehicle)) @ P(imu->antenna1)
        rotation_imu_vehicle = R.from_euler('zxy', rotation_imu_vehicle_angles, degrees=True).as_matrix()
        self.translation_vehicle_antenna1 = np.linalg.inv(rotation_imu_vehicle) @ translation_imu_antenna1

        # Reading the data into a DataFrame
        imu_data_df = read_data_into_dataframe(raw_data_file_path)

        date = raw_data_file_path.split('/')[-1]
        date = date.split('_')[1]
        year = int(date[0:4])
        month = int(date[4:6])
        day = int(date[6:8])

        # Applying the parse_line function to the DataFrame
        parsed_data_df = imu_data_df.apply(parse_line, axis=1, args=(year, month, day)).dropna()

        # Grouping the data by GPS time
        self.parsed_data_df = list(parsed_data_df.groupby('gps_time'))

        if ignore:
            delete_index = len(self.parsed_data_df) * 0.10
            del self.parsed_data_df[0: int(delete_index)]
        else:
            self.grouped_data_df = parsed_data_df

    def write_data_to_bag(self):
        # Assumption: the coordinate system of the antenna 1 is not well-defined.
        # During moving, the position of the antenna 1 can be determined in the local
        # level frame (East-North-Up). It follows definition
        # X: East, Y: North, Z: Up
        # At first, it is (0 0 0)^T. Then, it can be determined
        # by function like lla2enu(lla, lla_ref). And we always assume that the rotation between
        # antenna 1 and the local level frame is the identity matrix I.
        # Thus at first, the local level frame is exactly the antenna 1 frame, then between antenna 1 frame and the
        # local level frame there exist only translation.

        # For example, after several seconds, in the local level frame the position of the antenna 1 is T_p and
        # the rotation from the local level frame to the vehicle body frame is R1, and also the translation from
        # the vehicle body frame to the antenna 1 is T2, how to determine the position of the origin of the vehicle body
        # frame in the local level frame? i.e how to determine the translation from the local level frame to
        # the vehicle body frame T1.

        # Solution: the transform from local level frame to the antenna 1 is [I | T_p]
        # the transform from local level frame to the vehicle body frame is [R1 | T1]
        # the transform from the vehicle body frame to the antenna 1 is [R1^T | T2]
        # [I | T_p] = [R1 | T1] * [R1^T | T2] => T_p = R1 * T2 + T1
        # T1 = T_p - R1 * T2

        only_best = 0
        only_bestvel = 0
        only_bestgnss = 0
        only_bestgnssvel = 0
        for indx in range(len(self.parsed_data_df)):
            data = self.parsed_data_df[indx]
        for indx in range(len(self.grouped_data_df)):
            data = self.grouped_data_df[indx]
            gps_time_str = data[0].split(',')

            # print(type(data[1]))
            # <class 'pandas.core.frame.DataFrame'>

            # print(data[1].columns)
            # Index(['command_type', 'gps_time', 'raw_line'], dtype='object')

            stamp = gps_time_to_ros_time(gps_week=gps_time_str[0], seconds_in_week=gps_time_str[1])
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
                # self.get_logger.info(row['command_type'])
                # self.get_logger.info(row['raw_line'])
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
                        msg.header.stamp = stamp
                        msg.header.frame_id = 'antenna1'
                        msg.child_frame_id = 'vehicle'

                        # r is the rotation from antenna1 to the vehicle body frame
                        # self.translation_vehicle_antenna1 is the translation from the vehicle body frame to antenna 1
                        # let T1 = [R1 | T1] is the transform from the antenna 1 to the vehicle body frame
                        # T2 = [R2 | T2] is the transform from the vehicle body frame to the antenna 1
                        # T2 = inv(T1) where R2 = R1.T and T2 = -R1.T * T1
                        # in this case r = R1 and self.translation_vehicle_antenna1 = T2
                        # thus T1 = - R1 * T2 = -r * self.translation_vehicle_antenna1
                        r = R.from_quat([x, y, z, w]).as_matrix()
                        translation_base_to_child = - r @ self.translation_vehicle_antenna1

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
                        # this log contains redundant rotation information from local level frame (ENU)
                        # to vehicle body frame which is redundant to the log 'INSATTQSA',
                        # additionally this log contains the standard deviation of the roll, pitch, yaw angles
                        pass

                    case 'BESTXYZA':
                        xyz_msg = NavECEF()
                        xyz_msg.header.stamp = stamp
                        xyz_msg.header.frame_id = 'antenna1'
                        xyz_msg.pos_sol_status = data[0]
                        xyz_msg.pos_type = data[1]
                        xyz_msg.pos.x = float(data[2])
                        xyz_msg.pos.y = float(data[3])
                        xyz_msg.pos.z = float(data[4])
                        xyz_msg.pos_covariance[0] = float(data[5])
                        xyz_msg.pos_covariance[4] = float(data[6])
                        xyz_msg.pos_covariance[8] = float(data[7])
                        xyz_msg.vel_sol_status = data[8]
                        xyz_msg.vel_type = data[9]
                        xyz_msg.vel.x = float(data[10])
                        xyz_msg.vel.y = float(data[11])
                        xyz_msg.vel.z = float(data[12])
                        xyz_msg.vel_covariance[0] = float(data[13])
                        xyz_msg.vel_covariance[4] = float(data[14])
                        xyz_msg.vel_covariance[8] = float(data[15])
                        xyz_msg.base_station_id = data[16]
                        xyz_msg.diff_age = float(data[18])
                        xyz_msg.sol_age = float(data[19])
                        xyz_msg.nb_sat_tracked = int(data[20])
                        xyz_msg.nb_sat_solution = int(data[21])

                        self.writer.write(
                            '/bestxyz', serialize_message(xyz_msg),
                            Time.from_msg(stamp).nanoseconds)

                    case 'BESTUTMA':
                        utm_msg = NavUTM()
                        utm_msg.header.stamp = stamp
                        utm_msg.header.frame_id = 'antenna1'
                        utm_msg.pos_sol_status = data[0]
                        utm_msg.pos_type = data[1]
                        utm_msg.longitudinal_zone_num = int(data[2])
                        utm_msg.longitudinal_zone_letter = data[3]
                        utm_msg.northing = float(data[4])
                        utm_msg.easting = float(data[5])
                        utm_msg.height = float(data[6])
                        utm_msg.undulation = float(data[7])
                        utm_msg.pos_covariance[0] = float(data[9])
                        utm_msg.pos_covariance[4] = float(data[10])
                        utm_msg.pos_covariance[8] = float(data[11])
                        utm_msg.base_station_id = data[12]
                        utm_msg.diff_age = float(data[13])
                        utm_msg.sol_age = float(data[14])
                        utm_msg.nb_sat_tracked = int(data[15])
                        utm_msg.nb_sat_solution = int(data[16])

                        self.writer.write(
                            '/bestutm', serialize_message(utm_msg),
                            Time.from_msg(stamp).nanoseconds)

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

        self.get_logger().info(f'In msg, {only_bestgnss} only has bestgnsspos')
        self.get_logger().info(f'In msg, {only_bestgnssvel} only has bestgnssposvel')
        self.get_logger().info(f'In msg, {only_best} only has bestpos')
        self.get_logger().info(f'In msg, {only_bestvel} only has bestvel')


def main(args=None):
    rclpy.init(args=args)
    topic_infos = {'/bestgnss': 'novatel_sensor_fusion/msg/NavSatExtended',
                   '/best': 'novatel_sensor_fusion/msg/NavSatExtended',
                   '/gpgga': 'novatel_sensor_fusion/msg/NavSat',
                   '/gpggalong': 'novatel_sensor_fusion/msg/NavSat',
                   '/gprmc': 'novatel_sensor_fusion/msg/NavRMC',
                   '/imu': 'sensor_msgs/msg/Imu',
                   '/bestutm': 'novatel_sensor_fusion/msg/NavUTM',
                   '/bestxyz': 'novatel_sensor_fusion/msg/NavECEF'}


    imu_data_rate = 100

    # translation_imu2antenna1 = [dx, dy, dz] is the translation from IMU frame to
    # antenna1 frame, i.e. antenna1. Since after rotating X axis around 180 degrees
    # the IMU frame is transformed to the vehicle frame.
    # the translation from vehicle frame to the antenna 1 is [dx, -dy, -dz].

    # e.g.
    # SETINSTRANSLATION ANT1 0.431 0.0 -0.013 0.001 0.001 0.001 IMUBODY
    # SETINSTRANSLATION ANT2 -0.506 0.0 -0.013 0.001 0.001 0.001 IMUBODY
    #
    # SETINSTRANSLATION ANT1 0.431 0.0 0.013 0.001 0.001 0.001 VEHICLE
    # SETINSTRANSLATION ANT2 -0.506 0.0 0.013 0.001 0.001 0.001 VEHICLE

    # dx = 0.431
    # dy = 0.0
    # dz = -0.013

    # # Since the vehicle body frame is the default output frame if it is set
    # # thus the translation from vehicle body frame to the antenna 1 is preferred
    # translation = [dx, -dy, -dz]

    recorder = ImuRawDataBagRecorder(
        topic_infos=topic_infos,
        # raw_data_file_path=file_path,
        # bag_name=rosbag_name,
        imu_data_rate=imu_data_rate,
        ignore=False
    )

    recorder.write_data_to_bag()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
