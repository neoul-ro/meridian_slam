"""Relay FAST-LIVO2 odometry to the TF tree required by the platform.

FAST-LIVO2 publishes /aft_mapped_to_init (map -> aft_mapped, the IMU body
frame). The platform TF tree wants map -> base_link. This node applies the
fixed base_link->imu_link mount offset and broadcasts map -> base_link.

It also publishes that same pose on /pose, which is the topic the Meridian
pipeline consumes. FAST-LIVO2 used to publish /pose itself, but it only knows
the IMU frame -- the base_link offset lives here and in the URDF, so the two
would have had to be kept in step by hand.

The same pose goes out a second time on /pose_cov as a
PoseWithCovarianceStamped, which is what meridian_msgs/README.md asks an
Isometry3d to arrive as. /pose stays a PoseStamped so nothing that already
subscribes to it breaks.
"""

import math

import numpy as np

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import (PoseStamped, PoseWithCovarianceStamped,
                               TransformStamped)
from tf2_ros import TransformBroadcaster


def quat_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def quat_mult(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quat_conj(q):
    return (-q[0], -q[1], -q[2], q[3])


def rotate_vec(q, v):
    p = (v[0], v[1], v[2], 0.0)
    x, y, z, _ = quat_mult(quat_mult(q, p), quat_conj(q))
    return (x, y, z)


class OdomTfRelay(Node):
    def __init__(self):
        super().__init__('odom_tf_relay')
        self.declare_parameter('odom_topic', '/aft_mapped_to_init')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('pose_topic', '/pose')
        self.declare_parameter('pose_cov_topic', '/pose_cov')
        # Pose of imu_link expressed in base_link (must match the static
        # base_link -> chassis -> imu_link chain published by bringup).
        self.declare_parameter('imu_in_base_xyz', [0.0, 0.0, 0.0])
        self.declare_parameter('imu_in_base_rpy', [0.0, 0.0, 0.0])

        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        t_bi = self.get_parameter('imu_in_base_xyz').value
        rpy = self.get_parameter('imu_in_base_rpy').value
        q_bi = quat_from_rpy(*rpy)

        # Fixed inverse mount transform: imu_link -> base_link.
        self.q_ib = quat_conj(q_bi)
        tx, ty, tz = rotate_vec(self.q_ib, t_bi)
        self.t_ib = (-tx, -ty, -tz)

        self.tf_broadcaster = TransformBroadcaster(self)
        pose_topic = self.get_parameter('pose_topic').value
        self.pose_pub = self.create_publisher(PoseStamped, pose_topic, 10)
        pose_cov_topic = self.get_parameter('pose_cov_topic').value
        self.pose_cov_pub = self.create_publisher(
            PoseWithCovarianceStamped, pose_cov_topic, 10)
        self.warned_no_cov = False
        topic = self.get_parameter('odom_topic').value
        self.sub = self.create_subscription(Odometry, topic, self.on_odom, 10)
        self.get_logger().info(
            f'Relaying {topic} -> TF {self.map_frame} -> {self.base_frame}'
            f' and -> {pose_topic}, {pose_cov_topic}')

    def on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        q_mi = (o.x, o.y, o.z, o.w)  # map(=camera_init) -> imu body

        q_mb = quat_mult(q_mi, self.q_ib)
        off = rotate_vec(q_mi, self.t_ib)

        tf = TransformStamped()
        tf.header.stamp = msg.header.stamp
        tf.header.frame_id = self.map_frame
        tf.child_frame_id = self.base_frame
        tf.transform.translation.x = p.x + off[0]
        tf.transform.translation.y = p.y + off[1]
        tf.transform.translation.z = p.z + off[2]
        tf.transform.rotation.x = q_mb[0]
        tf.transform.rotation.y = q_mb[1]
        tf.transform.rotation.z = q_mb[2]
        tf.transform.rotation.w = q_mb[3]
        self.tf_broadcaster.sendTransform(tf)

        # Same pose, as a topic. The stamp is carried through from the odometry
        # unchanged: it is the LIO measurement time, and consumers pair this
        # pose with the camera frame it belongs to.
        pose = PoseStamped()
        pose.header = tf.header
        pose.pose.position.x = tf.transform.translation.x
        pose.pose.position.y = tf.transform.translation.y
        pose.pose.position.z = tf.transform.translation.z
        pose.pose.orientation = tf.transform.rotation
        self.pose_pub.publish(pose)

        # And again with the covariance, which is the form the Meridian
        # contract asks for. The odometry covariance describes the IMU pose and
        # this one describes base_link, so it does not carry over untouched: a
        # world-frame rotation error turns the whole rig about the IMU, and the
        # mount offset converts that into a position error at base_link. With
        # p_base = p_imu + off, dp_base = dp_imu - [off]x dtheta, so the 6x6
        # Jacobian is J = [[I, -[off]x], [0, I]] and C_base = J C_imu J'. off
        # is already in map coordinates from the transform above. The offset is
        # 38 cm, so at a realistic 1 deg of yaw uncertainty this term is 6.6 mm
        # -- not something to drop.
        cov = np.asarray(msg.pose.covariance, dtype=float).reshape(6, 6)
        pc = PoseWithCovarianceStamped()
        pc.header = tf.header
        pc.pose.pose = pose.pose
        if cov[0, 0] < 0.0 or not cov.any():
            # Upstream that predates the covariance being filled in publishes
            # 36 zeros, which a consumer reads as a perfectly known pose. Say
            # unknown instead, the way REP 103 does.
            pc.pose.covariance[0] = -1.0
            if not self.warned_no_cov:
                self.warned_no_cov = True
                self.get_logger().warn(
                    f'{self.get_parameter("odom_topic").value} carries no '
                    'covariance; publishing it as unknown (-1) rather than zero')
        else:
            skew = np.array([[0.0, -off[2], off[1]],
                             [off[2], 0.0, -off[0]],
                             [-off[1], off[0], 0.0]])
            j = np.eye(6)
            j[0:3, 3:6] = -skew
            pc.pose.covariance = (j @ cov @ j.T).ravel().tolist()
        self.pose_cov_pub.publish(pc)


def main(args=None):
    rclpy.init(args=args)
    node = OdomTfRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
