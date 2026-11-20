# Проверка самоколлизий робота в MuJoCo
# Берем ту же модель что у судьи (scene.xml) и список пар из pairs.csv (655 пар "checked")

import csv
import math
import mujoco
import numpy as np

JOINTS = ['FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint',
          'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint',
          'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint',
          'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint']
DEG = math.pi / 180


class Collision:
    def __init__(self, scene_file, pairs_file, max_depth=0.000999, step1=0.1 * DEG, step2=0.05 * DEG):
        # max_depth - сколько можно проникнуть: по ТЗ запрещено больше 1 мм
        # (0.999 мм - запас 1 микрон на округление углов во float32)
        # step1 - шаг как у судьи, step2 - еще мельче для надежности
        self.model = mujoco.MjModel.from_xml_path(scene_file)
        self.data = mujoco.MjData(self.model)
        self.max_depth = max_depth
        self.step1 = step1
        self.step2 = step2
        m = self.model

        # где в qpos лежат наши 12 суставов
        self.qadr = []
        for name in JOINTS:
            j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, name)
            if j < 0:
                raise RuntimeError('joint ' + name + ' not found in model')
            self.qadr.append(m.jnt_qposadr[j])
        self.qadr = np.array(self.qadr)

        # пол нам не нужен - выключаем
        for g in range(m.ngeom):
            if m.geom_bodyid[g] == 0:
                m.geom_contype[g] = 0
                m.geom_conaffinity[g] = 0

        # читаем пары из csv
        self.pairs = np.zeros((m.ngeom, m.ngeom), dtype=bool)
        self.n_pairs = 0
        with open(pairs_file) as f:
            for row in csv.DictReader(f):
                g1 = int(row['geom1_id'])
                g2 = int(row['geom2_id'])
                for g, body in [(g1, row['body1']), (g2, row['body2'])]:
                    if g >= m.ngeom:
                        raise RuntimeError('bad csv: geom ' + str(g) + ' not in model')
                    real = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, int(m.geom_bodyid[g]))
                    if real != body:
                        raise RuntimeError('bad csv: geom ' + str(g) + ' is on ' + real + ', csv says ' + body)
                if row['diagnostic_policy'] == 'checked':
                    self.pairs[g1, g2] = True
                    self.pairs[g2, g1] = True
                    self.n_pairs += 1
        if self.n_pairs == 0:
            raise RuntimeError('bad csv: no checked pairs')

        # пары которые mujoco сам не проверяет - для них считаем расстояние отдельно
        self.extra = []
        for g1 in range(m.ngeom):
            for g2 in range(g1 + 1, m.ngeom):
                if self.pairs[g1, g2] and not self.mujoco_checks(g1, g2):
                    self.extra.append((g1, g2))
        self.fromto = np.zeros(6)

    def mujoco_checks(self, g1, g2):
        # повторяет фильтры mujoco: маски, одно тело, родитель-потомок
        m = self.model
        if not ((m.geom_contype[g1] & m.geom_conaffinity[g2]) or (m.geom_contype[g2] & m.geom_conaffinity[g1])):
            return False
        b1 = m.body_weldid[m.geom_bodyid[g1]]
        b2 = m.body_weldid[m.geom_bodyid[g2]]
        if b1 == b2:
            return False
        if not (int(m.opt.disableflags) & int(mujoco.mjtDisableBit.mjDSBL_FILTERPARENT)):
            p1 = m.body_weldid[m.body_parentid[b1]]
            p2 = m.body_weldid[m.body_parentid[b2]]
            if b1 != 0 and b2 != 0 and (p1 == b2 or p2 == b1):
                return False
        return True

    def set_pose(self, q):
        self.data.qpos[:] = self.model.qpos0
        self.data.qpos[self.qadr] = q
        mujoco.mj_fwdPosition(self.model, self.data)

    def min_dist(self, q, only_pairs=True):
        # минимальное расстояние между деталями (минус = проникновение)
        self.set_pose(q)
        best = math.inf
        for i in range(self.data.ncon):
            c = self.data.contact[i]
            if only_pairs and not self.pairs[c.geom1, c.geom2]:
                continue
            if c.dist < best:
                best = float(c.dist)
        if only_pairs:
            for g1, g2 in self.extra:
                d = mujoco.mj_geomDistance(self.model, self.data, g1, g2, 0.01, self.fromto)
                if d < best:
                    best = float(d)
        return best

    def contacts(self, q):
        # все контакты (для тестов)
        self.set_pose(q)
        result = []
        for i in range(self.data.ncon):
            c = self.data.contact[i]
            result.append((int(c.geom1), int(c.geom2), float(c.dist)))
        return result

    def is_ok(self, q):
        return self.min_dist(q) >= -self.max_depth

    def path_ok(self, q1, q2):
        # проверяем все промежуточные позы от q1 до q2 (q1 уже проверена)
        q1 = np.array(q1, dtype=float)
        q2 = np.array(q2, dtype=float)
        diff = q2 - q1
        dist = np.max(np.abs(diff))
        if dist <= 1e-12:
            return self.is_ok(q2)
        if not self.is_ok(q2):
            return False
        n1 = max(1, math.ceil(dist / self.step1 - 1e-9))
        n2 = max(1, math.ceil(dist / self.step2 - 1e-9))
        points = set()
        for i in range(1, n1):
            points.add(i / n1)
        for i in range(1, n2):
            points.add(i / n2)
        for t in sorted(points):
            if not self.is_ok(q1 + diff * t):
                return False
        return True
