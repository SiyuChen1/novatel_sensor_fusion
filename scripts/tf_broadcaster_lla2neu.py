from geometry_msgs.msg import TransformStamped
from novatel_sensor_fusion.msg import NavSatStatusExtended
from novatel_sensor_fusion_py.lla2geodetic.lla2geodetic import lla2enu

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster


class LLA2ENUPublisher(Node):

    def __init__(self):
        super().__init__('lla2enu_tf2_frame_publisher')

        self.lat_ref = 50.77766795767
        self.lon_ref = 6.07832613239
        self.alt_ref = 178.5262

        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        self.subscription = self.create_subscription(
            NavSatStatusExtended,
            'bestgnsspos',
            self.handle_lla2enu,
            1)
        self.subscription  # prevent unused variable warning

    def handle_lla2enu(self, msg):
        t = TransformStamped()

        # Read message content and assign it to
        # corresponding tf variables
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'local_level_i'

        xyz = lla2enu(msg.latitude, msg.longitude, msg.altitude,
                      self.lat_ref, self.lon_ref, self.alt_ref, degrees=True)

        t.transform.translation.x = xyz[0]
        t.transform.translation.y = xyz[1]
        t.transform.translation.z = xyz[2]

        # assume that rotation between local_vehicle_i and base_link is identity

        # Send the transformation
        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = LLA2ENUPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
