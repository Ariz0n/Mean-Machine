# Логика блока безопасности T2 (без ROS, чтобы можно было тестировать)
# Порядок суставов: FR, FL, RR, RL, у каждой ноги hip, thigh, calf

import math
import numpy as np

DEG = math.pi / 180

# пределы суставов из ТЗ
Q_MIN = np.array([-0.863, -0.686, -2.818] * 4)
Q_MAX = np.array([0.863, 4.501, -0.888] * 4)

# поза стоя (если не пришел lowstate)
STAND = np.array([0.0, 0.8536, -1.7072] * 4)

# допустимые значения полей LowCmd
KP_MAX = 1000.0
KD_MAX = 100.0
TAU_MAX = 100.0
DQ_MAX = 20.0

STEP = 5 * DEG - 1e-5   # максимальный шаг выхода за одно сообщение: 5 градусов по ТЗ
                        # (минус 0.0006 градуса, чтобы после округления во float32 не стало 5.00003)
JUMP = 5 * DEG + 2e-5   # если цель прыгнула больше чем на 5 градусов - идем медленно
                        # (+0.001 градуса, чтобы шаг ровно 5 градусов после float32 не считался скачком)
SLOW_STEP = 0.5 * DEG   # шаг при медленном движении
MIN_MOVE = 0.01 * DEG   # если двигаться меньше этого - считаем что уперлись

TIMEOUT = 0.2           # через сколько секунд без команд включаем удержание
FREEZE = 0.4            # через сколько секунд цель точно перестает двигаться (по ТЗ 0.5)
WALL_TIMEOUT = 1.0      # на случай если время симуляции не идет

HOLD_KP = 40.0
HOLD_KD = 2.0
MARGIN = 0.0001         # отступ от пределов
BAD_Q = 50.0            # такой угол точно не угол (например 2.146e9)
VEL_STOP = 10000.0      # в Unitree это значит "скорость не задана"


def clamp_q(q):
    return np.clip(q, Q_MIN + MARGIN, Q_MAX - MARGIN)


def check_cmd(mode, q, dq, kp, kd, tau):
    """Проверка команды. Возвращает (ok, причина, cmd).
    cmd - словарь с массивами по 12 суставам."""
    q = np.array(q, dtype=float)
    dq = np.array(dq, dtype=float)
    kp = np.array(kp, dtype=float)
    kd = np.array(kd, dtype=float)
    tau = np.array(tau, dtype=float)
    mode = np.array(mode)

    if len(q) < 12 or len(dq) < 12 or len(kp) < 12 or len(kd) < 12 or len(tau) < 12 or len(mode) < 12:
        return False, 'too short', None

    # NaN или Inf в любом поле любого слота - отклоняем все сообщение
    for arr in [q, dq, kp, kd, tau]:
        if not np.all(np.isfinite(arr)):
            return False, 'nan or inf', None

    if not np.all(mode[:12] == 10):
        return False, 'mode is not 10', None

    q = q[:12]
    if np.any(np.abs(q) > BAD_Q):
        return False, 'q is not an angle', None

    cmd = {}
    cmd['q'] = clamp_q(q)
    cmd['clamped'] = bool(np.any(cmd['q'] != q))

    dq = dq[:12]
    dq = np.where(np.abs(dq) >= VEL_STOP, 0.0, dq)
    cmd['dq'] = np.clip(dq, -DQ_MAX, DQ_MAX)
    cmd['kp'] = np.clip(kp[:12], 0, KP_MAX)
    cmd['kd'] = np.clip(kd[:12], 0, KD_MAX)
    cmd['tau'] = np.clip(tau[:12], -TAU_MAX, TAU_MAX)
    return True, 'ok', cmd


def hold_cmd(q):
    zero = np.zeros(12)
    return {'q': np.array(q, dtype=float), 'dq': zero.copy(), 'kp': np.full(12, HOLD_KP),
            'kd': np.full(12, HOLD_KD), 'tau': zero.copy()}


class Safety:
    def __init__(self, checker):
        # checker должен уметь is_ok(q) и path_ok(q1, q2)
        self.checker = checker
        self.q_out = None      # что мы публикуем сейчас
        self.target = None     # последняя принятая цель
        self.cmd = None        # последняя принятая команда
        self.last_sim = None   # когда пришла последняя команда (время симуляции)
        self.last_wall = None  # то же, но реальное время
        self.slow = False      # идем к цели медленно (был скачок)
        self.stuck = False     # уперлись в коллизию
        self.count = {'accepted': 0, 'clamped': 0, 'rejected': 0, 'blocked': 0}
        self.reasons = {}

    def ready(self):
        return self.q_out is not None

    def start(self, q):
        q = clamp_q(np.array(q, dtype=float))
        self.q_out = q.copy()
        self.target = q.copy()
        self.cmd = hold_cmd(q)
        self.slow = False
        self.stuck = False

    def reject(self, reason):
        self.count['rejected'] += 1
        self.reasons[reason] = self.reasons.get(reason, 0) + 1
        return False, reason

    def new_cmd(self, mode, q, dq, kp, kd, tau, t_sim, t_wall):
        ok, reason, cmd = check_cmd(mode, q, dq, kp, kd, tau)
        if not ok:
            return self.reject(reason)
        if not self.ready():
            return self.reject('not started')
        if not self.checker.is_ok(cmd['q']):
            return self.reject('self collision')

        # новая цель - можно снова пробовать двигаться
        if self.stuck and not np.array_equal(cmd['q'], self.target):
            self.stuck = False

        jump = np.max(np.abs(cmd['q'] - self.target))
        if jump > JUMP:
            self.slow = True
        elif np.max(np.abs(self.target - self.q_out)) <= JUMP:
            self.slow = False

        self.target = cmd['q'].copy()
        self.cmd = cmd
        self.last_sim = t_sim
        self.last_wall = t_wall
        self.count['accepted'] += 1
        if cmd['clamped']:
            self.count['clamped'] += 1
        return True, 'ok'

    def no_input_time(self, t_sim, t_wall):
        # сколько секунд нет команд (по времени симуляции, как в ТЗ)
        if self.last_sim is None:
            return float('inf')
        dt_sim = t_sim - self.last_sim
        dt_wall = t_wall - self.last_wall
        if dt_sim < 0 or t_sim <= 0:
            dt_sim = dt_wall
        if dt_wall > WALL_TIMEOUT:
            return max(dt_sim, dt_wall)
        return dt_sim

    def input_lost(self, t_sim, t_wall):
        return self.no_input_time(t_sim, t_wall) > TIMEOUT

    def correcting(self):
        # выход еще не дошел до цели из-за скачка или препятствия
        return self.slow or self.stuck or np.max(np.abs(self.target - self.q_out)) > JUMP

    def move(self):
        # двигаем выход к цели, проверяя коллизии по пути
        if self.stuck:
            return
        diff = self.target - self.q_out
        dist = np.max(np.abs(diff))
        if dist <= 1e-12:
            return
        step = SLOW_STEP if self.slow else STEP
        part = min(1.0, step / dist)
        if self.checker.path_ok(self.q_out, self.q_out + diff * part):
            self.q_out = self.q_out + diff * part
            return

        # на пути коллизия - ищем сколько можно пройти (делим пополам)
        self.count['blocked'] += 1
        lo = 0.0
        hi = part
        for i in range(8):
            mid = (lo + hi) / 2
            if self.checker.path_ok(self.q_out, self.q_out + diff * mid):
                lo = mid
            else:
                hi = mid
        if lo * dist > MIN_MOVE:
            self.q_out = self.q_out + diff * lo
        else:
            self.stuck = True

    def get_cmd(self, t_sim, t_wall):
        # что отправить в /lowcmd сейчас
        if not self.ready():
            raise RuntimeError('safety not started')

        wait = self.no_input_time(t_sim, t_wall)
        if self.last_sim is not None and wait > TIMEOUT:
            self.slow = False                  # быстро доходим до цели
            if wait > FREEZE:
                self.target = self.q_out.copy()  # дальше не двигаемся

        self.move()
        self.q_out = clamp_q(self.q_out)
        q = self.q_out.copy()
        zero = np.zeros(12)

        if self.input_lost(t_sim, t_wall):
            # удержание: dq=0, tau=0, kp и kd из последней команды если нормальные
            kp = self.cmd['kp']
            kd = self.cmd['kd']
            kp = np.where((kp >= 5) & (kp <= KP_MAX), kp, HOLD_KP)
            kd = np.where((kd >= 0.1) & (kd <= KD_MAX), kd, HOLD_KD)
            out = {'q': q, 'dq': zero.copy(), 'kp': kp, 'kd': kd, 'tau': zero.copy()}
        elif self.correcting():
            # исправляем опасную команду - ее dq и tau не передаем
            out = {'q': q, 'dq': zero.copy(), 'kp': self.cmd['kp'].copy(), 'kd': self.cmd['kd'].copy(),
                   'tau': zero.copy()}
        else:
            out = {'q': q, 'dq': self.cmd['dq'].copy(), 'kp': self.cmd['kp'].copy(),
                   'kd': self.cmd['kd'].copy(), 'tau': self.cmd['tau'].copy()}

        # на всякий случай проверяем выход еще раз
        for name in ['q', 'dq', 'kp', 'kd', 'tau']:
            if not np.all(np.isfinite(out[name])):
                return hold_cmd(q)
        out['dq'] = np.clip(out['dq'], -DQ_MAX, DQ_MAX)
        out['kp'] = np.clip(out['kp'], 0, KP_MAX)
        out['kd'] = np.clip(out['kd'], 0, KD_MAX)
        out['tau'] = np.clip(out['tau'], -TAU_MAX, TAU_MAX)
        return out
