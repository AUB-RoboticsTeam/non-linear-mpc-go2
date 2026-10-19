#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """Extract yaw from a quaternion."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quaternion(yaw: float):
    """Create a quaternion containing yaw only."""
    half_yaw = yaw / 2.0

    qx = 0.0
    qy = 0.0
    qz = math.sin(half_yaw)
    qw = math.cos(half_yaw)

    return qx, qy, qz, qw


class MapToBaseLinkTfBroadcaster(Node):

    def __init__(self):
        super().__init__('map_to_base_link_tf_broadcaster')

        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('parent_frame', 'map')
        self.declare_parameter('child_frame', 'base')

        odom_topic = self.get_parameter('odom_topic').value
        self.parent_frame = self.get_parameter('parent_frame').value
        self.child_frame = self.get_parameter('child_frame').value

        self.broadcaster = TransformBroadcaster(self)

        self.subscription = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            QoSPresetProfiles.SENSOR_DATA.value
        )

        self.get_logger().info(
            f'Publishing dynamic TF: '
            f'{self.parent_frame} -> {self.child_frame} from {odom_topic}'
        )

    def odom_callback(self, msg: Odometry):
        orientation = msg.pose.pose.orientation

        yaw = quaternion_to_yaw(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
        )

        qx, qy, qz, qw = yaw_to_quaternion(yaw)

        transform = TransformStamped()

        transform.header.stamp = msg.header.stamp
        transform.header.frame_id = self.parent_frame
        transform.child_frame_id = self.child_frame

        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = 0.0

        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw

        self.broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)

    node = MapToBaseLinkTfBroadcaster()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
