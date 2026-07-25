import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from meridian_msgs.msg import RGBDFrame, PoseEstimate


class SlamNode(Node):

    def __init__(self):
        super().__init__('meridian_slam')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10)

        self.pose_pub = self.create_publisher(PoseEstimate, '/pose_estimate', qos)
        self.frame_sub = self.create_subscription(
            RGBDFrame, '/rgbd_frame', self.frame_callback, qos)

        self.frame_count = 0

        self.get_logger().info('meridian_slam started: /rgbd_frame -> /pose_estimate')

    def frame_callback(self, msg):
        # V1 policy: one PoseEstimate per frame, identity pose, no retroactive revision.
        pose_msg = PoseEstimate()
        pose_msg.timestamp = msg.timestamp
        pose_msg.world_t_camera.pose.orientation.w = 1.0
        # position and covariance stay at their zero defaults
        pose_msg.has_covariance = False
        pose_msg.has_trajectory_revision = False
        pose_msg.trajectory_revision = 0

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
