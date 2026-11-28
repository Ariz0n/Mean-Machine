#!/usr/bin/env python3
# Узел map -> odom для T5
#
# По ТЗ: старт робота - это точка (0, 0) карты map, курс 0, вперед - ось +X.
# Одометрия (odom/odom_node.py) начинает с точки (0, 0), но курс берет из IMU как есть,
# поэтому map -> odom - это поворот на минус начальный курс.
# Если Judge присылает начальную позу в /initialpose (пока робот на старте) - ставим робота в эту позу.
# Положение без дрейфа уже дает одометрия по лидару, отдельная локализация не нужна.
# Если одометрия перезапустилась (поза в odom скачком вернулась в 0) - пересчитываем map -> odom,
# чтобы робот остался там же на карте.

import math

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import StaticTransformBroadcaster

MOVED = 0.5   # робот уехал со старта дальше этого - /initialpose больше не принимаем, м
JUMP = 1.0    # между двумя сообщениями /odom (0.02 с) робот столько не проходит - значит одометрия перезапустилась, м


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class MapOdom(Node):
    def __init__(self):
        super().__init__('map_odom', parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.odom = None          # последняя поза робота в odom: (x, y, yaw)
        self.start = None         # поза в odom на старте
        self.tr = None            # текущее map -> odom: (tx, ty, a)
        self.tf = StaticTransformBroadcaster(self)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        # /initialpose обычно шлют reliable; transient_local тоже примем
        for durability in (DurabilityPolicy.VOLATILE, DurabilityPolicy.TRANSIENT_LOCAL):
            qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST,
                             depth=5, durability=durability)
            self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.on_initialpose, qos)
        self.get_logger().info('map_odom started: map = start pose, waiting for /odom')

    def in_map(self, pose):
        # поза из odom -> в map по текущему map -> odom
        tx, ty, a = self.tr
        x, y, yaw = pose
        c, s = math.cos(a), math.sin(a)
        return tx + c * x - s * y, ty + s * x + c * y, yaw + a

    def on_odom(self, msg):
        p = msg.pose.pose
        pose = (p.position.x, p.position.y, yaw_of(p.orientation))
        if self.tr is None:
            # первая поза: робот стоит в начале карты, курс 0
            self.odom = self.start = pose
            self.set_pose(0.0, 0.0, 0.0, msg.header.stamp)
            return
        if math.hypot(pose[0] - self.odom[0], pose[1] - self.odom[1]) > JUMP:
            # одометрия перезапустилась: робот там же, где был на карте
            x, y, yaw = self.in_map(self.odom)
            self.get_logger().warn('odometry restarted, keeping robot pose on the map')
            self.odom = pose
            self.set_pose(x, y, yaw, msg.header.stamp)
            return
        self.odom = pose

    def on_initialpose(self, msg):
        if self.odom is None:
            self.get_logger().warn('initial pose before odometry, ignored')
            return
        # начальная поза - это поза на старте. Если робот уже уехал, поздняя "начальная поза"
        # сдвинула бы всю карту - такое не принимаем
        if math.hypot(self.odom[0] - self.start[0], self.odom[1] - self.start[1]) > MOVED:
            self.get_logger().warn('initial pose ignored: robot already moved from the start')
            return
        p = msg.pose.pose
        x, y, yaw = p.position.x, p.position.y, yaw_of(p.orientation)
        if not all(math.isfinite(v) for v in (x, y, yaw)):
            return
        self.set_pose(x, y, yaw, self.get_clock().now().to_msg())

    def set_pose(self, x, y, yaw, stamp):
        """Сделать map -> odom таким, чтобы робот сейчас был в точке (x, y) с курсом yaw в map."""
        ox, oy, oyaw = self.odom
        a = yaw - oyaw                       # поворот odom относительно map
        c, s = math.cos(a), math.sin(a)
        tx = x - (c * ox - s * oy)           # сдвиг odom относительно map
        ty = y - (s * ox + c * oy)
        self.tr = (tx, ty, a)
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        t.transform.translation.x = tx
        t.transform.translation.y = ty
        t.transform.rotation.z = math.sin(a / 2)
        t.transform.rotation.w = math.cos(a / 2)
        self.tf.sendTransform(t)
        self.get_logger().info(f'map -> odom: x {tx:.3f}, y {ty:.3f}, yaw {math.degrees(a):.1f} deg '
                               f'(robot in map: {x:.2f}, {y:.2f}, {math.degrees(yaw):.1f} deg)')


def main():
    rclpy.init()
    node = MapOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
