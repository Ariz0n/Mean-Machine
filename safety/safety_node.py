#!/usr/bin/env python3
# ROS 2 узел блока безопасности T2
# /lowcmd_bare -> проверка -> /lowcmd (200 раз в секунду)
# /lowstate нужен только чтобы узнать начальную позу

import os
import sys
import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from unitree_go.msg import LowCmd, LowState

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from safety import Safety, STAND
from collision import Collision

SCENE = os.environ.get('SCENE_FILE', '/opt/mujoco_menagerie/unitree_go1/scene.xml')
PAIRS = os.environ.get('PAIRS_FILE', '/opt/t2/safety/pairs.csv')
RATE = 200.0


class SafetyNode(Node):
    def __init__(self):
        # время берем из симуляции (/clock)
        super().__init__('safety_node', parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])

        self.checker = Collision(SCENE, PAIRS)
        self.safety = Safety(self.checker)
        self.start_time = time.monotonic()
        self.last_print = time.monotonic()
        self.last_warn = 0.0
        self.sent = 0

        # best effort подписка работает и с reliable и с best effort издателями
        qos_in = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
        qos_out = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(LowCmd, '/lowcmd_bare', self.on_cmd, qos_in)
        self.create_subscription(LowState, '/lowstate', self.on_state, qos_in)
        self.pub = self.create_publisher(LowCmd, '/lowcmd', qos_out)

        # таймер по обычным часам, чтобы публиковать всегда, даже если /clock тормозит
        self.create_timer(1.0 / RATE, self.on_timer, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info(f'safety node started, rate {RATE:.0f} hz, {self.checker.n_pairs} collision pairs')

    def sim_time(self):
        return self.get_clock().now().nanoseconds / 1e9

    def on_state(self, msg):
        if self.safety.ready():
            return
        q = [msg.motor_state[i].q for i in range(12)]
        if np.all(np.isfinite(q)):
            self.safety.start(q)
            self.get_logger().info('got first lowstate, holding current pose')

    def on_cmd(self, msg):
        mode, q, dq, kp, kd, tau = [], [], [], [], [], []
        for m in msg.motor_cmd:
            mode.append(m.mode)
            q.append(m.q)
            dq.append(m.dq)
            kp.append(m.kp)
            kd.append(m.kd)
            tau.append(m.tau)
        ok, reason = self.safety.new_cmd(mode, q, dq, kp, kd, tau, self.sim_time(), time.monotonic())
        if not ok and time.monotonic() - self.last_warn > 1.0:
            self.last_warn = time.monotonic()
            self.get_logger().warn('rejected command: ' + reason)

    def on_timer(self):
        if not self.safety.ready():
            # ждем lowstate 1 секунду, потом берем позу стоя
            if time.monotonic() - self.start_time < 1.0:
                return
            self.safety.start(STAND)
            self.get_logger().warn('no lowstate, using stand pose')

        out = self.safety.get_cmd(self.sim_time(), time.monotonic())
        msg = LowCmd()
        msg.head = [0xFE, 0xEF]
        msg.level_flag = 0xFF
        for i in range(12):
            msg.motor_cmd[i].mode = 10
            msg.motor_cmd[i].q = float(out['q'][i])
            msg.motor_cmd[i].dq = float(out['dq'][i])
            msg.motor_cmd[i].kp = float(out['kp'][i])
            msg.motor_cmd[i].kd = float(out['kd'][i])
            msg.motor_cmd[i].tau = float(out['tau'][i])
        self.pub.publish(msg)
        self.sent += 1

        # раз в 5 секунд печатаем статистику
        if time.monotonic() - self.last_print > 5.0:
            self.last_print = time.monotonic()
            c = self.safety.count
            self.get_logger().info(f'sent {self.sent}, accepted {c["accepted"]}, clamped {c["clamped"]}, '
                                   f'rejected {c["rejected"]} {self.safety.reasons}, blocked {c["blocked"]}')


def main():
    rclpy.init()
    node = SafetyNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
