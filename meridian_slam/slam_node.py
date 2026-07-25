import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Image


class SlamNode(Node):

    def __init__(self):
        super().__init__('meridian_slam')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10)

        self.declare_parameter('world_frame_id', 'map')
        self.world_frame_id = self.get_parameter('world_frame_id').value

        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/pose', qos)
        self.frame_sub = self.create_subscription(
            Image, '/camera/rgb', self.frame_callback, qos)

        self.frame_count = 0

        self.get_logger().info('meridian_slam started: /camera/rgb -> /pose')

    def frame_callback(self, msg):
        # V1 policy: one pose per frame, identity pose, no retroactive revision.
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = msg.header.stamp
        pose_msg.header.frame_id = self.world_frame_id
        pose_msg.pose.pose.orientation.w = 1.0
        # position and covariance stay at their zero defaults

        self.pose_pub.publish(pose_msg)

        self.frame_count += 1
        self.get_logger().info(
            'processed %d frames' % self.frame_count,
            throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = SlamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
