#!/usr/bin/env python3

# this script shows that the order of writing ros messages doesn't affect the order of replaying rosbag
# only the timestamp matters

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.serialization import serialize_message
import rosbag2_py

from std_msgs.msg import Header


class SimpleBagRecorder(Node):
    def __init__(self, nb, topic_name):
        super().__init__('simple_bag_recorder')
        self.writer = rosbag2_py.SequentialWriter()

        storage_options = rosbag2_py._storage.StorageOptions(
            uri='bag_simple_header',
            storage_id='mcap')
        converter_options = rosbag2_py._storage.ConverterOptions('', '')
        self.writer.open(storage_options, converter_options)

        topic_info = rosbag2_py._storage.TopicMetadata(
            name=topic_name,
            type='std_msgs/msg/Header',
            serialization_format='cdr')
        self.writer.create_topic(topic_info)

        start = self.get_clock().now()

        # the bigger msg.frame_id, the smaller msg.stamp
        # the order of writing msg into rosbag doesn't affect the order when replaying the rosbag data,
        # only the timestamp used for writing the rosbag is related

        # msg with smaller frame_id but bigger timestamp will be written
        # into rosbag file firstly.
        for idx in range(nb):
            now = start - Duration(seconds=idx, nanoseconds=0)
            msg = Header()
            sec_nanosec = now.seconds_nanoseconds()
            msg.stamp.sec = sec_nanosec[0]
            msg.stamp.nanosec = sec_nanosec[1]
            msg.frame_id = str(idx)
            self.writer.write(
                topic_name,
                serialize_message(msg),
                now.nanoseconds)


def main(args=None):
    rclpy.init(args=args)
    SimpleBagRecorder(nb=100, topic_name='/simple_header')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
