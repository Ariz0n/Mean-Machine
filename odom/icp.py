# Карта и наложение скана на карту
#
# Карта - это точки стен и предметов, которые лидар уже видел (на плоскости, в системе odom).
# Новый скан нужно сдвинуть так, чтобы его точки легли на стены карты.
# Этот сдвиг и есть положение робота.
#
# Поворот робота уже известен из IMU, поэтому ищем только сдвиг по x и y.

import numpy as np
from scipy.spatial import cKDTree   # быстрый поиск ближайшей точки

CELL = 0.03          # оставляем по одной точке на клетку 3 x 3 см
MAX_DIST = 0.25      # точку скана дальше 25 см от карты не используем, м
STEPS = 15           # сколько раз уточняем сдвиг
MIN_POINTS = 40      # если совпало меньше точек - результату не верим
GOOD_ERROR = 0.02    # точки с ошибкой больше 2 см учитываем слабее, м


def thin_out(points):
    # прореживание: по одной точке в каждой клетке CELL x CELL
    if len(points) == 0:
        return points
    cells = np.floor(points / CELL).astype(np.int64)
    _, first = np.unique(cells, axis=0, return_index=True)
    return points[np.sort(first)]


def wall_directions(points):
    """Для каждой точки находим, куда смотрит стена (нормаль - направление поперек стены).
    Берем 6 ближайших соседей. Если они лежат на одной линии - это стена, нормаль известна.
    Если нет (угол, мусор) - точку не используем."""
    tree = cKDTree(points)
    k = min(6, len(points))
    _, neighbours = tree.query(points, k=k)

    near = points[neighbours]                       # соседи каждой точки
    dx = near[:, :, 0] - near[:, :, 0].mean(axis=1, keepdims=True)
    dy = near[:, :, 1] - near[:, :, 1].mean(axis=1, keepdims=True)
    # разброс соседей по x и y (матрица 2 x 2 для каждой точки)
    spread = np.zeros((len(points), 2, 2))
    spread[:, 0, 0] = (dx * dx).sum(axis=1)
    spread[:, 0, 1] = spread[:, 1, 0] = (dx * dy).sum(axis=1)
    spread[:, 1, 1] = (dy * dy).sum(axis=1)
    # самое узкое направление разброса - поперек стены
    sizes, directions = np.linalg.eigh(spread)
    normal = directions[:, :, 0]
    # стена: вдоль линии разброс большой, поперек - маленький
    is_wall = (sizes[:, 1] > 1e-6) & (sizes[:, 0] < 0.1 * sizes[:, 1])
    return normal, is_wall


class Map:
    def __init__(self):
        self.points = np.zeros((0, 2))
        self.normals = np.zeros((0, 2))
        self.tree = None

    def empty(self):
        return self.tree is None or len(self.points) < MIN_POINTS

    def add(self, points):
        # добавить точки скана (уже в системе odom)
        all_points = thin_out(np.vstack([self.points, points]))
        normal, is_wall = wall_directions(all_points)
        self.points = all_points[is_wall]
        self.normals = normal[is_wall]
        self.tree = cKDTree(self.points) if len(self.points) else None

    def match(self, scan, guess):
        """Ищем сдвиг, при котором scan + сдвиг лучше всего ложится на стены карты.
        guess - с какого сдвига начинать. Возвращает (сдвиг, сколько точек совпало),
        сдвиг = None, если наложить не получилось."""
        shift = np.array(guess, dtype=float)
        found = 0
        for step in range(STEPS):
            moved = scan + shift
            # ближайшая точка карты для каждой точки скана
            dist, nearest = self.tree.query(moved, distance_upper_bound=MAX_DIST)
            ok = np.isfinite(dist)
            found = int(ok.sum())
            if found < MIN_POINTS:
                return None, found

            normal = self.normals[nearest[ok]]
            # насколько точка скана не дошла до стены (вдоль нормали)
            error = np.sum((moved[ok] - self.points[nearest[ok]]) * normal, axis=1)
            # точки с большой ошибкой учитываем слабее (это могут быть выбросы)
            weight = np.where(np.abs(error) < GOOD_ERROR, 1.0, GOOD_ERROR / np.maximum(np.abs(error), 1e-9))

            # метод наименьших квадратов: какой сдвиг уменьшит все ошибки сразу
            A = normal * weight[:, None]
            H = A.T @ normal
            if np.linalg.cond(H) > 1e4:
                # видна только одна стена: вдоль нее сдвиг не определить
                return None, found
            correction = -np.linalg.solve(H, A.T @ error)
            shift = shift + correction
            if np.linalg.norm(correction) < 1e-4:
                break   # дальше не уточняется
        return shift, found
