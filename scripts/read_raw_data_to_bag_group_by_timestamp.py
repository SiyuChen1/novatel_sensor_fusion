#!/usr/bin/env python3

from novatel_sensor_fusion.msg import NavRMC, NavSat, NavSatExtended
from novatel_sensor_fusion_py.raw_data_to_bag.data2rosmsg import gps_time_to_ros_header
from novatel_sensor_fusion_py.ultilies.group_raw_data_by_timestamp import read_data_into_dataframe, parse_line
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.serialization import serialize_message
import rosbag2_py

from std_msgs.msg import Header
from sensor_msgs.msg import Imu


class ImuRawDataBagRecorder(Node):
    def __init__(self, topic_infos, raw_data_file_path):
        super().__init__('imu_raw_data_bag_recorder')
        # self.writer = rosbag2_py.SequentialWriter()
        #
        # storage_options = rosbag2_py._storage.StorageOptions(
        #     uri='imu_raw_data_bag_recorder',
        #     storage_id='mcap')
        # converter_options = rosbag2_py._storage.ConverterOptions('', '')
        # self.writer.open(storage_options, converter_options)
        #
        # for topic_name in topic_infos.keys():
        #     topic_type = topic_infos[topic_name]
        #     topic_info = rosbag2_py._storage.TopicMetadata(
        #         name=topic_name,
        #         type=topic_type,
        #         serialization_format='cdr')
        #     self.writer.create_topic(topic_info)

        # Reading the data into a DataFrame
        imu_data_df = read_data_into_dataframe(raw_data_file_path)

        # Applying the parse_line function to the DataFrame
        parsed_data_df = imu_data_df.apply(parse_line, axis=1).dropna()

        # Grouping the data by GPS time
        self.grouped_data_df = list(parsed_data_df.groupby('gps_time'))

    def write_data_to_bag(self):
        for indx in range(len(self.grouped_data_df)):
            data = self.grouped_data_df[indx]
            gps_time_str = data[0]

            # print(type(data[1]))
            # <class 'pandas.core.frame.DataFrame'>

            print(data[1].columns)
            # Index(['command_type', 'gps_time', 'raw_line'], dtype='object')

            # len(data[1]) is the number of raw data lines
            for line_id in len(data[1]):
                pass


def main(args=None):
    rclpy.init(args=args)
    topic_infos = {'/bestgnss': 'novatel_sensor_fusion/msg/NavSatExtended',
                   '/best': 'novatel_sensor_fusion/msg/NavSatExtended',
                   '/gpgga': 'novatel_sensor_fusion/msg/NavSat',
                   '/gpggalong': 'novatel_sensor_fusion/msg/NavSat',
                   '/gprmc': 'novatel_sensor_fusion/msg/NavRMC',
                   '/imu': 'sensor_msgs/msg/Imu'}

    file_path = '/home/siyuchen/catkin_ws/src/novatel_sensor_fusion' \
                '/jupyter_notebook/data/small_raw_imu_data.txt'

    recorder = ImuRawDataBagRecorder(topic_infos=topic_infos, raw_data_file_path=file_path)
    recorder.write_data_to_bag()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
