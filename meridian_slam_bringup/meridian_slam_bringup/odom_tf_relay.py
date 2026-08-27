"""Relay FAST-LIVO2 odometry to the TF tree required by the platform.

FAST-LIVO2 publishes /aft_mapped_to_init (map -> aft_mapped, the IMU body
frame). The platform TF tree wants map -> base_link. This node applies the
fixed base_link->imu_link mount offset and broadcasts map -> base_link.

It also publishes that same pose on /pose, which is the topic the Meridian
pipeline consumes. FAST-LIVO2 used to publish /pose itself, but it only knows
the IMU frame.

The mount offset is read from TF rather than configured. imu_link -> base_link
runs entirely through the static URDF chain (base_link -> chassis -> imu_link)
and does not touch map, so looking it up here is not circular even though this
node is what publishes map -> base_link. It used to be a pair of parameters
copied out of the URDF by hand, which is one more place for the rig geometry to
drift out of step.

The same pose goes out a second time on /pose_cov as a
PoseWithCovarianceStamped, which is what meridian_msgs/README.md asks an
Isometry3d to arrive as. /pose stays a PoseStamped so nothing that already
subscribes to it breaks.
"""

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import (PoseStamped, PoseWithCovarianceStamped,
                               TransformStamped)
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener


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
        self.declare_parameter('imu_frame', 'imu_link')

        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.imu_frame = self.get_parameter('imu_frame').value

        # imu_link -> base_link, filled in on the first odometry message once
        # robot_state_publisher has latched the static chain.
        self.q_ib = None
        self.t_ib = None
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self, spin_thread=True)
        self.mount_warnings = 0

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

    def mount_ready(self):
        """Latch imu_link -> base_link from TF. False until the URDF is up.

        Nothing is published until this succeeds. Falling back to an assumed
        offset would put base_link 38 cm from where it belongs and say nothing
        about it, and a pose that is quietly wrong costs more than one that is
        visibly missing.
        """
        if self.q_ib is not None:
            return True
        try:
            tf = self.buffer.lookup_transform(self.imu_frame, self.base_frame, Time())
        except TransformException as exc:
            self.mount_warnings += 1
            if self.mount_warnings in (1, 50) or self.mount_warnings % 200 == 0:
                self.get_logger().warn(
                    f'waiting for {self.imu_frame} -> {self.base_frame} on TF, '
                    f'nothing published yet ({exc})')
            return False
        t, r = tf.transform.translation, tf.transform.rotation
        self.t_ib = (t.x, t.y, t.z)
        self.q_ib = (r.x, r.y, r.z, r.w)
        self.get_logger().info(
            f'{self.imu_frame} -> {self.base_frame} from TF: '
            f'[{t.x:.4f}, {t.y:.4f}, {t.z:.4f}]')
        return True

    def on_odom(self, msg: Odometry):
        if not self.mount_ready():
            return
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
