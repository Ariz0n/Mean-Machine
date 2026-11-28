#!/usr/bin/env python3
# ROS 2 узел одометрии T4
#
# Слушает:  /imu/data (поворот робота), /scan (лидар)
# Публикует: /odom и TF odom -> trunk, 50 раз в секунду по времени симуляции
# Роботом не управляет.

import math
import os
import sys

import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from odom import Odom, rotation_matrix, yaw_of

RATE = 50.0   # сколько раз в секунду публикуем (по ТЗ не меньше 20)


class OdomNode(Node):
    def __init__(self):
        # время берем из симуляции (/clock), как требует ТЗ
        super().__init__('odom_node', parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])

        self.odom = Odom()
        self.gyro = np.zeros(3)      # угловая скорость из IMU (для twist)
        self.last_stamp = None       # время последней публикации
        self.last_print = None
        self.sent = 0

        # скан считается несколько миллисекунд - даем ему свой поток,
        # чтобы IMU и публикация не ждали
        scan_thread = MutuallyExclusiveCallbackGroup()
        main_thread = MutuallyExclusiveCallbackGroup()

        # best effort подписка работает и с reliable и с best effort издателями
        imu_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=20)
        scan_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=2)
        self.create_subscription(Imu, '/imu/data', self.on_imu, imu_qos, callback_group=main_thread)
        self.create_subscription(LaserScan, '/scan', self.on_scan, scan_qos, callback_group=scan_thread)

        self.pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf = TransformBroadcaster(self)

        # T5: точки скана сразу в системе odom (без пола) для карт стоимости Nav2.
        # В T4 выключено (параметр cloud по умолчанию false).
        self.declare_parameter('cloud', False)
        self.cloud_pub = None
        if self.get_parameter('cloud').value:
            self.cloud_pub = self.create_publisher(PointCloud2, '/scan_cloud', 10)
        self.create_timer(1.0 / RATE, self.on_timer, callback_group=main_thread)   # таймер по времени симуляции

        self.get_logger().info(f'odom node started, rate {RATE:.0f} hz sim time, imu orientation + lidar icp')

    def on_imu(self, msg):
        o = msg.orientation
        self.odom.imu([o.x, o.y, o.z, o.w])
        w = msg.angular_velocity
        gyro = np.array([w.x, w.y, w.z])
        if np.all(np.isfinite(gyro)):
            self.gyro = gyro

    def on_scan(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        try:
            self.odom.scan(t, msg.ranges, msg.angle_min, msg.angle_increment)
        except Exception as e:
            # один плохой скан не должен ронять узел
            self.get_logger().warn(f'scan failed: {e!r}', throttle_duration_sec=5.0)
            return
        if self.cloud_pub is not None and self.odom.cloud_time == t and len(self.odom.cloud):
            header = Header(stamp=msg.header.stamp, frame_id='odom')
            self.cloud_pub.publish(point_cloud2.create_cloud_xyz32(header, self.odom.cloud.tolist()))

    def on_timer(self):
        now = self.get_clock().now()
        t = now.nanoseconds / 1e9
        if t <= 0:
            return   # /clock еще не пришел
        if self.last_stamp is not None and now.nanoseconds <= self.last_stamp:
            return   # время не сдвинулось - второй раз с тем же временем не публикуем

        pose = self.odom.pose(t)
        if pose is None:
            return   # еще не было IMU
        p, q = pose
        if not (np.all(np.isfinite(p)) and np.all(np.isfinite(q))):
            return   # NaN и Inf не публикуем никогда
        q = q / np.linalg.norm(q)
        self.last_stamp = now.nanoseconds
        stamp = now.to_msg()

        # /odom
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'trunk'
        msg.pose.pose.position.x = float(p[0])
        msg.pose.pose.position.y = float(p[1])
        msg.pose.pose.position.z = float(p[2])
        msg.pose.pose.orientation.x = float(q[0])
        msg.pose.pose.orientation.y = float(q[1])
        msg.pose.pose.orientation.z = float(q[2])
        msg.pose.pose.orientation.w = float(q[3])
        # примерная точность (дисперсии) по x, y, z, крену, тангажу, курсу
        cov = [0.0] * 36
        for i, value in enumerate([1e-4, 1e-4, 1e-2, 1e-4, 1e-4, 1e-4]):
            cov[i * 7] = value
        msg.pose.covariance = cov
        # скорость - в системе корпуса trunk
        v = rotation_matrix(q).T @ np.array([self.odom.vel[0], self.odom.vel[1], 0.0])
        if np.all(np.isfinite(v)):
            msg.twist.twist.linear.x = float(v[0])
            msg.twist.twist.linear.y = float(v[1])
            msg.twist.twist.linear.z = float(v[2])
        msg.twist.twist.angular.x = float(self.gyro[0])
        msg.twist.twist.angular.y = float(self.gyro[1])
        msg.twist.twist.angular.z = float(self.gyro[2])
        msg.twist.covariance = cov

        # TF odom -> trunk: та же поза и то же время, что в /odom
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'trunk'
        tf.transform.translation.x = msg.pose.pose.position.x
        tf.transform.translation.y = msg.pose.pose.position.y
        tf.transform.translation.z = msg.pose.pose.position.z
        tf.transform.rotation = msg.pose.pose.orientation

        self.pub.publish(msg)
        self.tf.sendTransform(tf)
        self.sent += 1

        # раз в 5 секунд печатаем статистику
        if self.last_print is None or t - self.last_print > 5.0:
            self.last_print = t
            c = self.odom.count
            self.get_logger().info(f'sent {self.sent}, x {p[0]:.3f}, y {p[1]:.3f}, yaw {math.degrees(yaw_of(q)):.1f} deg, '
                                   f'scans {c["scans"]}, matched {c["matched"]}, failed {c["failed"]}, '
                                   f'keyframes {c["keyframes"]}, map {len(self.odom.map.points)} pts')


def main():
    rclpy.init()
    node = OdomNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
