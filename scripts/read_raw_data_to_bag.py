#!/usr/bin/env python3

# https://roboticsbackend.com/ros2-package-for-both-python-and-cpp-nodes/
# This is something you don’t have to do with a standard ROS2 Python package
# (it’s managed for you), but with this setup if you don’t add this line,
# you will get an error when you try to start the node with
import re

from novatel_sensor_fusion_py.raw_data_to_bag.data2rosmsg import data_to_gnss_msg, \
    data_to_tf_paras, data_to_imu
from rclpy.node import Node
from rclpy.serialization import serialize_message
from rclpy.time import Time

import rosbag2_py

IMU_DATA_RATE = 100

# SETINSROTATION RBV 180 0 0
# https://docs.novatel.com/OEM7/Content/PDFs/OEM7_Commands_Logs_Manual.pdf page 1071
# Use the SETINSROTATION command to specify rotational offsets between the
# IMU frame and other reference frames, such as the vehicle frame or an ALIGN baseline.
# Offsets must be entered as the rotation
# from the IMU body frame, to the frame of interest. The order of rotations is Z, X, Y.
# All rotations are right-handed.
# RBV: Rotation from the IMU body frame to the vehicle frame.

# translation_imu2ll = [dx, dy, dz] is the translation from IMU frame to
# instantaneous local level frame (ENU), i.e. antenna1. Since after rotating 180 degree around 180 degrees
# the IMU frame is transformed to the vehicle frame.
# the translation from vehicle frame to instantaneous local level frame (ENU) is [dx, -dy, -dz]
dx = 0.431
dy = 0.0
dz = 0.013
translation_imu2ll = [dx, dy, dz]
translation = [dx, -dy, -dz]
topic_dict = {'BESTGNSSPOSA': ('bestgnsspos', 'novatel_sensor_fusion/msg/NavSatStatusExtended',
                               data_to_gnss_msg, 'local_level_i'),
              'BESTPOSA': ('bestpos', 'novatel_sensor_fusion/msg/NavSatStatusExtended',
                           data_to_gnss_msg, 'local_level_i'),
              'INSATTQSA': ('tf', 'tf2_msgs/msg/TFMessage', data_to_tf_paras,
                            ('local_level_i', 'vehicle', translation)),
              'CORRIMUSA': ('imu', 'sensor_msgs/msg/Imu', data_to_imu, (IMU_DATA_RATE, 'vehicle'))}


class RawData2Bag(Node):

    def __init__(self, data_path, bag_name):
        self.data_path = data_path
        self.writer = rosbag2_py.SequentialWriter()
        storage_options = rosbag2_py._storage.StorageOptions(
            uri=bag_name,
            storage_id='mcap')
        converter_options = rosbag2_py._storage.ConverterOptions('', '')
        self.writer.open(storage_options, converter_options)

        for ele in topic_dict.keys():
            topic_info = rosbag2_py._storage.TopicMetadata(
                name=topic_dict[ele][0],
                type=topic_dict[ele][1],
                serialization_format='cdr')
            print(ele, topic_dict[ele])
            self.writer.create_topic(topic_info)

        self.start_reading()

    def start_reading(self):
        with open(self.data_path) as my_file:
            content = my_file.read().split('\n')
            for index, line in enumerate(content):
                if len(line) > 0:
                    infos = line.split(';')
                    if len(infos) - 2 == 0:
                        header_ = infos[0].split(',')
                        data_ = infos[1].split(',')
                        m = re.search(r'(?<=(%|#))\w+', header_[0])
                        key = m.group(0)
                        if key in topic_dict.keys():
                            topic_name = topic_dict[key][0]
                            fun = topic_dict[key][2]
                            paras = topic_dict[key][3]
                            msg = fun(header_, data_, paras)
                            if msg:
                                try:
                                    time_stamp = msg.header.stamp
                                except Exception:
                                    time_stamp = msg.transforms[0].header.stamp
                                self.writer.write(topic_name,
                                                  serialize_message(msg),
                                                  Time.from_msg(time_stamp).nanoseconds)
                    elif len(infos) - 1 == 0:
                        # log gpgga ontime 1
                        # log gpggalong ontime 1
                        # log gprmc ontime 1
                        # log gphdt onchanged
                        pass
                    else:
                        pass

                    # if "BESTGNSSPOS" in line:
                    #
                    # if "BESTPOS" in line:
                    #     best_pos_msg = data_to_gnss_msg(line)
                    #     if best_pos_msg:
                    #         self.writer.write('bestpos',
                    #                           serialize_message(best_pos_msg),
                    #                           Time.from_msg(best_pos_msg.header.stamp).nanoseconds)
        print('publishing finished')


def main(args=None):
    data_path = '/home/siyuchen/Documents/' + \
        'Novatel_Stereocam_Daten_20230703/20230703_140222/IMUData_20230703_140222.log'
    bag_name = 'novatel_sensor_data'
    RawData2Bag(data_path, bag_name)


if __name__ == '__main__':
    main()
