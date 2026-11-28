#!/usr/bin/env python3
# Распознавание ArUco для T5
#
# Смотрит в 5 камер, ищет метки словаря DICT_4X4_50 и 5 раз в секунду публикует в /aruco/detections
# метки, которые видны сейчас: {"ids": [3, 7]}. "Сейчас" - любая камера видела метку за последние 0.5 с.
# Публикуем только то, что реально нашли на картинке.

import json

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String

CAMERAS = ['/camera', '/camera_chin', '/camera_left', '/camera_right', '/camera_rearDown']
RATE = 5.0           # сколько раз в секунду публикуем
KEEP = 0.5           # сколько секунд метка считается "видной сейчас"
MIN_SIDE = 12        # меньше стороны в пикселях - не верим (слишком далеко, похоже на шум)
EVERY = 0.15         # кадр одной камеры обрабатываем не чаще, с (экономим процессор)


class ArucoNode(Node):
    def __init__(self):
        super().__init__('aruco_node', parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.params = cv2.aruco.DetectorParameters_create()
        self.seen = {}           # id -> когда последний раз видели (время симуляции)
        self.last_frame = {}     # камера -> когда последний раз обрабатывали кадр
        self.all_ids = set()     # для статистики в логе
        self.pub = self.create_publisher(String, '/aruco/detections', 10)
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        for cam in CAMERAS:
            self.create_subscription(Image, cam + '/image_raw', lambda m, c=cam: self.on_image(m, c), qos)
        self.create_timer(1.0 / RATE, self.on_timer)
        self.get_logger().info('aruco node started: DICT_4X4_50, cameras ' + ' '.join(CAMERAS))

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def on_image(self, msg, cam):
        t = self.now()
        if t - self.last_frame.get(cam, -1e9) < EVERY:
            return
        self.last_frame[cam] = t
        gray = to_gray(msg)
        if gray is None:
            return
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.dictionary, parameters=self.params)
        if ids is None:
            return
        for c, i in zip(corners, ids.flatten()):
            side = np.mean(np.linalg.norm(c[0] - np.roll(c[0], 1, axis=0), axis=1))
            if side < MIN_SIDE or not 0 <= i < 50:
                continue
            self.seen[int(i)] = t
            if int(i) not in self.all_ids:
                self.all_ids.add(int(i))
                self.get_logger().info(f'new marker {int(i)} ({cam}, side {side:.0f} px), total {len(self.all_ids)}')

    def on_timer(self):
        t = self.now()
        ids = sorted(i for i, ts in self.seen.items() if t - ts <= KEEP)
        msg = String()
        msg.data = json.dumps({'ids': ids})
        self.pub.publish(msg)


def to_gray(msg):
    # картинка ROS -> серое изображение numpy
    if msg.encoding in ('rgb8', 'bgr8'):
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)[:, :msg.width * 3]
        img = img.reshape(msg.height, msg.width, 3)
        code = cv2.COLOR_RGB2GRAY if msg.encoding == 'rgb8' else cv2.COLOR_BGR2GRAY
        return cv2.cvtColor(img, code)
    if msg.encoding in ('mono8', '8UC1'):
        return np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)[:, :msg.width].copy()
    return None


def main():
    rclpy.init()
    node = ArucoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
