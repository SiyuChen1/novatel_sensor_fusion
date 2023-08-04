#!/usr/bin/env python3

import numpy as np
from novatel_sensor_fusion.msg import NavSatStatusExtended
from novatel_sensor_fusion_py.filter_impl import ExtendedKalmanFilter16States
from novatel_sensor_fusion_py.lla2geodetic.lla2geodetic import lla2enu
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Imu
from tf2_msgs.msg import TFMessage


class GNSSIMUSubscriber(Node):

    def __init__(self, imu_topic, imu_paras, gnss_topic):
        super().__init__('gnss_imu_subscriber')

        self.ekf = ExtendedKalmanFilter16States(
            imu_paras['imu_data_rate'], imu_paras['gyro_bias_instability'],
            imu_paras['accelerometer_bias_instability'],
            imu_paras['angular_random_walk'],
            imu_paras['velocity_random_walk']
        )

        self.init_quaternion_msg = None
        self.init_state = np.zeros(16)
        self.current_time = None
        self.is_ready = False
        self.is_gnss_ready = False
        self.lla_ref = [50.77766817103, 6.07832598964, 178.5169]

        self.imu_subscription = self.create_subscription(
            Imu,
            imu_topic,
            self.imu_callback,
            100)
        self.imu_subscription  # prevent unused variable warning

        self.quaternion_subscription = self.create_subscription(
            TFMessage,
            '/tf',
            self.quaternion_callback,
            100
        )
        self.quaternion_subscription

        self.gnss_subscription = self.create_subscription(
            NavSatStatusExtended,
            gnss_topic,
            self.gnss_callback,
            100
        )

    def gnss_callback(self, msg):
        lat = msg.latitude
        lon = msg.longitude
        alt = msg.altitude
        enu = lla2enu(lat, lon, alt, self.lla_ref[0], self.lla_ref[1], self.lla_ref[2])
        if not self.is_gnss_ready:
            self.init_state[4:7] = enu
            self.current_time = Time(seconds=msg.header.stamp.sec,
                                     nanoseconds=msg.header.stamp.nanosec)
            self.is_gnss_ready = True
        else:
            lat = msg.latitude
            lon = msg.longitude
            alt = msg.altitude
            enu = lla2enu(lat, lon, alt, self.lla_ref[0], self.lla_ref[1], self.lla_ref[2])
            self.ekf.fuse_gps()
            # TODO

    def imu_callback(self, msg):
        # self.get_logger().info('I heard Imu: "%s"' % msg.header.frame_id)
        if self.is_ready:
            accel = np.zeros(3)
            accel[0] = msg.linear_acceleration.x
            accel[1] = msg.linear_acceleration.y
            accel[2] = msg.linear_acceleration.z

            gyro = np.zeros(3)
            gyro[0] = msg.angular_velocity.x
            gyro[1] = msg.angular_velocity.y
            gyro[2] = msg.angular_velocity.z

            now = Time(seconds=msg.header.stamp.sec,
                       nanoseconds=msg.header.stamp.nanosec)
            dt = now - self.current_time
            print(dt.nanoseconds / 1e9)
            self.ekf.compute_next_state(accel, gyro, dt.nanoseconds / 1e9, True)
            self.current_time = now

    def quaternion_callback(self, msg):
        # self.get_logger().info('I heard Quaternion: "%s"' % msg.transforms[0].child_frame_id)
        if self.is_gnss_ready and not self.is_ready:
            init_quaternion_msg = msg.transforms[0]
            self.init_state[0] = init_quaternion_msg.transform.rotation.w
            self.init_state[1] = init_quaternion_msg.transform.rotation.x
            self.init_state[2] = init_quaternion_msg.transform.rotation.y
            self.init_state[3] = init_quaternion_msg.transform.rotation.z
            self.ekf.set_state(self.init_state)
            self.is_ready = True


def main(args=None):
    rclpy.init(args=args)

    imu_paras = {'imu_data_rate': 100,
                 # gyro_bias_noise refers to bias instability in the datasheet
                 # https://docs.novatel.com/OEM7/Content/PDFs/CPT7_Installation_Operation_Manual.pdf
                 # page 179 table 29, with unit degree/hour
                 'gyro_bias_instability': 0.45,

                 # gyro_noise refers to angular random walk in the datasheet
                 # https://docs.novatel.com/OEM7/Content/PDFs/CPT7_Installation_Operation_Manual.pdf
                 # page 179 table 29, with unit degree per sqrt(hour)
                 'angular_random_walk': 0.06,

                 # accelerometer_bias_noise refers to bias instability in the datasheet
                 # https://docs.novatel.com/OEM7/Content/PDFs/CPT7_Installation_Operation_Manual.pdf
                 # page 179 table 29, with unit mg, i.e. 0.001 * g
                 'accelerometer_bias_instability': 0.075,

                 # accelerometer_noise refers to velocity random walk in the datasheet
                 # https://docs.novatel.com/OEM7/Content/PDFs/CPT7_Installation_Operation_Manual.pdf
                 # page 179 table 29, with unit m/s per sqrt(hour)
                 'velocity_random_walk': 0.06
                 }

    subscriber = GNSSIMUSubscriber('/imu', imu_paras, '/bestgnsspos')

    rclpy.spin(subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
