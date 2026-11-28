# Одометрия T4 - вся логика (без ROS)
#
# Поворот робота берем из IMU.
# Положение x, y находим по лидару: накладываем скан на карту стен (icp.py).

import math
import numpy as np

from icp import Map, thin_out

# где стоит лидар относительно корпуса trunk (go1.urdf.xacro: base_scan xyz="0.22 0 0.11")
LIDAR_POS = np.array([0.22, 0.0, 0.11])

NEAR = 0.15             # ближе лидар врет (его минимум 0.12), м
FAR = 15.0              # дальше не берем, м
NEW_PLACE = 0.20        # добавляем скан в карту, если робот сдвинулся на столько, м
NEW_TURN = math.radians(10)   # или повернулся на столько
MAX_STEP = 0.15         # больше за один скан (0.05 с) робот сдвинуться не может, м
MAX_AHEAD = 0.1         # между сканами досчитываем по скорости не дольше этого, с
TRUNK_HEIGHT = 0.29     # высота корпуса над полом, когда собака стоит (для облака точек T5), м
FLOOR = 0.12            # точка скана ниже этого над полом - это пол, а не препятствие, м
CLOUD_RANGE = 6.0       # в облако для Nav2 берем точки не дальше этого от лидара, м


def rotation_matrix(q):
    # кватернион (x, y, z, w) -> матрица поворота 3 x 3
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def yaw_of(q):
    # курс (поворот вокруг вертикали) из кватерниона, рад
    x, y, z, w = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def angle_diff(a, b):
    # разница углов от -pi до pi
    return (a - b + math.pi) % (2 * math.pi) - math.pi


def check_quat(q):
    # нормальный кватернион: 4 числа без NaN и длина около 1. Возвращаем его нормированным или None
    q = np.array(q, dtype=float)
    if len(q) != 4 or not np.all(np.isfinite(q)):
        return None
    length = np.linalg.norm(q)
    if abs(length - 1.0) > 0.1:
        return None
    return q / length


class Odom:
    def __init__(self):
        self.q = None             # поворот робота из IMU (x, y, z, w)
        self.pos = np.zeros(2)    # положение x, y в системе odom (старт = 0, 0)
        self.vel = np.zeros(2)    # скорость x, y, м/с
        self.scan_time = None     # время последнего скана, с
        self.map = Map()
        self.map_pos = None       # где робот был, когда последний раз пополняли карту
        self.map_yaw = None
        self.count = {'scans': 0, 'matched': 0, 'failed': 0, 'keyframes': 0}
        self.points3d = np.zeros((0, 3))   # точки текущего скана (относительно робота) без пола
        self.cloud = None         # точки последнего скана в odom (x, y, z) без пола - для карт Nav2 в T5
        self.cloud_time = None

    def imu(self, q):
        # пришел поворот из IMU. Битый кватернион пропускаем - остается прошлый
        q = check_quat(q)
        if q is None:
            return False
        self.q = q
        return True

    def scan_to_points(self, ranges, angle_min, angle_step):
        # скан лидара -> точки на плоскости в осях odom (пока без сдвига робота)
        r = np.array(ranges, dtype=float)
        angle = angle_min + angle_step * np.arange(len(r))
        good = np.isfinite(r) & (r > NEAR) & (r < FAR)   # NaN, Inf, слишком близко и далеко - выкидываем
        r = r[good]
        angle = angle[good]
        # точка в системе лидара -> в системе корпуса
        points = np.stack([r * np.cos(angle), r * np.sin(angle), np.zeros(len(r))], axis=1) + LIDAR_POS
        # поворачиваем как повернут корпус (с наклоном - при ходьбе корпус качается)
        points = points @ rotation_matrix(self.q).T
        # для карт Nav2 (T5): точки выше пола. При качке луч через несколько метров задевает пол -
        # такие точки не препятствия
        self.points3d = points[(points[:, 2] + TRUNK_HEIGHT > FLOOR) & (r < CLOUD_RANGE)]
        return thin_out(points[:, :2])

    def scan(self, t, ranges, angle_min, angle_step):
        # пришел скан лидара в момент t
        if self.q is None:
            return False   # без IMU не знаем, как повернуть скан
        self.count['scans'] += 1
        yaw = yaw_of(self.q)
        points = self.scan_to_points(ranges, angle_min, angle_step)
        if len(points) < 40:
            return False

        # первый скан - начало карты, робот в точке (0, 0)
        if self.map.empty():
            self.map.add(points + self.pos)
            self.map_pos = self.pos.copy()
            self.map_yaw = yaw
            self.scan_time = t
            self.count['keyframes'] += 1
            return True

        # где робот должен быть, если едет с той же скоростью
        dt = 0.0 if self.scan_time is None else min(max(t - self.scan_time, 0.0), 0.5)
        guess = self.pos + self.vel * dt

        # накладываем скан на карту
        new_pos, found = self.map.match(points, guess)
        matched = new_pos is not None and np.linalg.norm(new_pos - self.pos) <= MAX_STEP + 0.5 * dt
        if matched:
            self.count['matched'] += 1
        else:
            # не наложился - верим прогнозу по скорости
            self.count['failed'] += 1
            new_pos = guess

        # скорость сглаживаем, чтобы не дергалась
        if dt > 1e-3:
            self.vel = 0.7 * self.vel + 0.3 * (new_pos - self.pos) / dt
        self.pos = new_pos
        self.scan_time = t
        # в системе odom корпус на высоте z = 0, поэтому z точек - высота относительно корпуса
        self.cloud = self.points3d + np.array([self.pos[0], self.pos[1], 0.0])
        self.cloud_time = t

        # робот сдвинулся или повернулся достаточно - добавляем скан в карту
        moved = np.linalg.norm(self.pos - self.map_pos) > NEW_PLACE
        turned = abs(angle_diff(yaw, self.map_yaw)) > NEW_TURN
        if matched and (moved or turned):
            self.map.add(points + self.pos)
            self.map_pos = self.pos.copy()
            self.map_yaw = yaw
            self.count['keyframes'] += 1
        return True

    def pose(self, t):
        """Поза на момент t: ((x, y, z), (qx, qy, qz, qw)) или None, если IMU еще не было.
        Между сканами (0.05 с) досчитываем сдвиг по скорости."""
        if self.q is None:
            return None
        p = self.pos.copy()
        if self.scan_time is not None:
            p = p + self.vel * min(max(t - self.scan_time, 0.0), MAX_AHEAD)
        return np.array([p[0], p[1], 0.0]), self.q.copy()   # z = 0: пол ровный
