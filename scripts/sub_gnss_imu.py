#!/usr/bin/env python3

import numpy as np
from novatel_sensor_fusion.msg import NavSatExtended
from novatel_sensor_fusion_py.filter_impl import ExtendedKalmanFilter16States
from novatel_sensor_fusion_py.lla2geodetic.lla2geodetic import enu2lla, lla2enu
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Imu


class GNSSIMUSubscriber(Node):

    def __init__(self, imu_paras):
        super().__init__('gnss_imu_subscriber')

        self.declare_parameter('use_gnss_velocity', False)
        self.use_gnss_velocity = self.get_parameter('use_gnss_velocity').get_parameter_value().bool_value
        self.declare_parameter('imu_topic_name', '/imu')
        self.imu_topic = self.get_parameter('imu_topic_name').get_parameter_value().string_value
        self.declare_parameter('best_topic_name', '/best')
        self.best_topic = self.get_parameter('best_topic_name').get_parameter_value().string_value
        self.declare_parameter('bestgnss_topic_name', '/bestgnss')
        self.gnss_topic = self.get_parameter('bestgnss_topic_name').get_parameter_value().string_value
        self.declare_parameter('imu_ekf_fused_topic_name', '/fused')
        self.fused_topic = self.get_parameter('imu_ekf_fused_topic_name').get_parameter_value().string_value

        self.declare_parameter('init_state_covariance', 1e-2)
        init_state_covariance = self.get_parameter('init_state_covariance').get_parameter_value().double_value
        self.declare_parameter('other_additive_noise', 1e-4)
        other_additive_noise = self.get_parameter('other_additive_noise').get_parameter_value().double_value

        self.get_logger().info(f'use gnss velocity {self.use_gnss_velocity}')

        self.ekf = ExtendedKalmanFilter16States(
            imu_sample_rate=imu_paras['imu_data_rate'],
            gyro_bias_instability=imu_paras['gyro_bias_instability'],
            accelerometer_bias_instability=imu_paras['accelerometer_bias_instability'],
            angular_random_walk=imu_paras['angular_random_walk'],
            velocity_random_walk=imu_paras['velocity_random_walk'],
            init_state_covariance=init_state_covariance * np.eye(16),
            other_additive_noise=other_additive_noise
        )

        self.init_quaternion_msg = None
        self.init_state = np.zeros(16)
        self.current_time = None
        self.is_quaternion_init = False
        self.is_gnss_ready = False

        # self.lla_ref = [50.77766817103, 6.07832598964, 178.5169]

        self.imu_subscription = self.create_subscription(
            Imu,
            self.imu_topic,
            self.imu_callback,
            100)
        self.imu_subscription  # prevent unused variable warning

        self.gnss_subscription = self.create_subscription(
            NavSatExtended,
            self.gnss_topic,
            self.gnss_callback,
            100
        )
        self.gnss_subscription

        self.best_subscription = self.create_subscription(
            NavSatExtended,
            self.best_topic,
            self.best_callback,
            100
        )
        self.best_subscription

        self.publisher = self.create_publisher(NavSatExtended, self.fused_topic, 10)

        self.get_logger().info('node is initialised')

    def gnss_callback(self, msg):
        if self.is_gnss_ready and self.is_quaternion_init:
            lat = msg.latitude
            lon = msg.longitude
            alt = msg.altitude
            enu = lla2enu(lat, lon, alt, self.lla_ref[0], self.lla_ref[1], self.lla_ref[2], degrees=True)
            v_h = msg.horizontal_speed
            theta = msg.direction_to_north * np.pi / 180
            v_north = v_h * np.cos(theta)
            v_east = v_h * np.sin(theta)
            v_up = msg.vertical_speed
            # self.get_logger().info('vertical speed %s ' % v_up)
            # msg.position_covariance[4]: longitude std
            cov_p = [msg.position_covariance[4],
                     msg.position_covariance[0],
                     msg.position_covariance[8]]
            cov_v = 5 * np.array(cov_p)
            if self.use_gnss_velocity:
                self.ekf.fuse_gps_position_velocity(enu, [v_east, v_north, v_up], cov_p, cov_v)
            else:
                self.ekf.fuse_gps_position(enu, cov_p)
            state = self.ekf.get_state()
            fused_msg = NavSatExtended()
            fused_msg.header = msg.header
            lla = enu2lla(state[4], state[5], state[6],
                          self.lla_ref[0], self.lla_ref[1], self.lla_ref[2], degrees=True)
            fused_msg.latitude = lla[0]
            fused_msg.longitude = lla[1]
            fused_msg.altitude = lla[2]

            # self.get_logger().info('alt in raw gnss message: %s' % alt)
            # self.get_logger().info('alt in fused message: %s' % lla[2])
            # self.get_logger().info('alt difference: %s ' % np.abs(alt - lla[2]))

            self.publisher.publish(fused_msg)

    def imu_callback(self, msg):
        # self.get_logger().info('I heard Imu: "%s"' % msg.header.frame_id)
        if self.is_gnss_ready:
            if not self.is_quaternion_init:
                # self.get_logger().info('quaternion is initialised')
                self.init_state[0] = msg.orientation.w
                self.init_state[1] = msg.orientation.x
                self.init_state[2] = msg.orientation.y
                self.init_state[3] = msg.orientation.z
                self.current_time = Time(seconds=msg.header.stamp.sec,
                                         nanoseconds=msg.header.stamp.nanosec)
                self.ekf.set_state(self.init_state)
                self.is_quaternion_init = True
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
            # self.get_logger().info(f"elapsed time: {dt.nanoseconds / 1e9}")
            if dt.nanoseconds < 0:
                self.get_logger().info('fatal error')
            self.ekf.imu_predict(accel, gyro, dt.nanoseconds / 1e9, True)
            # state = self.ekf.get_state()
            # self.get_logger().info(f'predicted (%0.8f, %0.8f, %0.8f)' % (state[4], state[5], state[6]))
            self.current_time = now

    def best_callback(self, msg):
        if not self.is_gnss_ready:
            lat = msg.latitude
            lon = msg.longitude
            alt = msg.altitude
            self.lla_ref = [lat, lon, alt]
            enu = lla2enu(lat, lon, alt, self.lla_ref[0],
                          self.lla_ref[1], self.lla_ref[2], degrees=True)

            # lla = enu2lla(enu[0], enu[1], enu[2],
            #               self.lla_ref[0], self.lla_ref[1], self.lla_ref[2], degrees=True)
            # self.get_logger().info(f'(%0.8f, %0.8f, %0.8f)' % (lla[0], lla[1], lla[2]))
            # self.get_logger().info(f'(%0.8f, %0.8f, %0.8f)' % (lat, lon, alt))
            # self.get_logger().info(f'(%0.8f, %0.8f, %0.8f)' % (enu[0], enu[1], enu[2]))

            self.init_state[4:7] = enu
            self.is_gnss_ready = True

    # def quaternion_callback(self, msg):
    #     # self.get_logger().info('I heard Quaternion: "%s"' % msg.transforms[0].child_frame_id)
    #     if self.is_gnss_ready and not self.is_quaternion_init:
    #         init_quaternion_msg = msg.transforms[0]
    #         self.init_state[0] = init_quaternion_msg.transform.rotation.w
    #         self.init_state[1] = init_quaternion_msg.transform.rotation.x
    #         self.init_state[2] = init_quaternion_msg.transform.rotation.y
    #         self.init_state[3] = init_quaternion_msg.transform.rotation.z
    #         self.ekf.set_state(self.init_state)
    #         self.is_quaternion_init = True


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

                 'velocity_random_walk': 0.06,

                 # Gyroscope Rate Random Walk (rad/s^1.5), determined by allan variance
                 # see imu_paras.md
                 'gyro_rate_random_walk': 1.71e-7,

                 # Accelerometer Acceleration Random Walk (m/s^2.5), determined by allan variance
                 # see imu_paras.md
                 'accelerometer_acceleration_random_walk': 1.56e-5,

                 'velocity_random_walk': 0.06
                 }

    subscriber = GNSSIMUSubscriber(imu_paras)

    rclpy.spin(subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
