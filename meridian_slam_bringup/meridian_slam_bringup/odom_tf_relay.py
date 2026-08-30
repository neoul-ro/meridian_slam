"""FAST-LIVO2 오도메트리를 플랫폼이 요구하는 TF 트리로 릴레이한다.

FAST-LIVO2 는 /aft_mapped_to_init (map -> aft_mapped, IMU 바디 프레임)을
발행한다. 플랫폼 TF 트리가 원하는 것은 map -> base_link 다. 이 노드가 고정된
base_link->imu_link 장착 오프셋을 적용해서 map -> base_link 를 브로드캐스트한다.

같은 자세를 /pose 로도 발행한다. Meridian 파이프라인이 소비하는 토픽이다.
예전에는 FAST-LIVO2 가 직접 /pose 를 냈지만, 그쪽은 IMU 프레임밖에 모른다.

장착 오프셋은 설정값이 아니라 TF 에서 읽는다. imu_link -> base_link 는 URDF 의
정적 체인(base_link -> chassis -> imu_link)만 타고 map 을 거치지 않으므로,
map -> base_link 를 발행하는 게 이 노드 자신이어도 순환이 아니다. 예전에는
URDF 값을 손으로 베낀 파라미터 두 개였는데, 그러면 리그 형상이 어긋날 자리가
하나 더 생긴다.

같은 자세가 /pose_cov 로 PoseWithCovarianceStamped 형태로 한 번 더 나간다.
meridian_msgs/README.md 가 Isometry3d 를 그 타입으로 받으라고 적고 있기
때문이다. /pose 는 PoseStamped 로 남겨서 이미 구독 중인 쪽이 깨지지 않게 한다.
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

        # imu_link -> base_link. robot_state_publisher 가 정적 체인을 래치한 뒤
        # 첫 오도메트리 메시지에서 채워진다.
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
        """imu_link -> base_link 를 TF 에서 래치한다. URDF 가 올라오기 전엔 False.

        이게 성공하기 전까지는 아무것도 발행하지 않는다. 가정한 오프셋으로
        대체하면 base_link 가 제자리에서 38 cm 벗어난 채 아무 말도 안 하게 되는데,
        조용히 틀린 자세는 눈에 띄게 없는 자세보다 비싸다.
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

        # 같은 자세를 토픽으로. 스탬프는 오도메트리 것을 그대로 넘긴다 — LIO
        # 측정 시각이고, 소비자는 이 자세를 그에 해당하는 카메라 프레임과
        # 짝지어 쓴다.
        pose = PoseStamped()
        pose.header = tf.header
        pose.pose.position.x = tf.transform.translation.x
        pose.pose.position.y = tf.transform.translation.y
        pose.pose.position.z = tf.transform.translation.z
        pose.pose.orientation = tf.transform.rotation
        self.pose_pub.publish(pose)

        # 공분산까지 붙여서 한 번 더. Meridian 계약이 요구하는 형태다.
        # 오도메트리 공분산은 IMU 자세를 기술하고 이건 base_link 를 기술하므로
        # 그대로 옮길 수 없다. 월드 프레임 회전 오차는 리그 전체를 IMU 기준으로
        # 돌리고, 장착 오프셋이 그걸 base_link 의 위치 오차로 바꾼다.
        # p_base = p_imu + off 이므로 dp_base = dp_imu - [off]x dtheta 이고,
        # 6x6 야코비안은 J = [[I, -[off]x], [0, I]], C_base = J C_imu J' 다.
        # off 는 위 변환에서 이미 map 좌표로 들어와 있다. 오프셋이 38 cm 라
        # 현실적인 yaw 불확실성 1 도에서 이 항이 6.6 mm 다 — 버릴 크기가 아니다.
        cov = np.asarray(msg.pose.covariance, dtype=float).reshape(6, 6)
        pc = PoseWithCovarianceStamped()
        pc.header = tf.header
        pc.pose.pose = pose.pose
        if cov[0, 0] < 0.0 or not cov.any():
            # 공분산을 채우기 전 버전의 upstream 은 0 을 36 개 내보내는데,
            # 소비자는 그걸 완벽히 아는 자세로 읽는다. REP 103 방식대로
            # 모른다고 말해 준다.
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
