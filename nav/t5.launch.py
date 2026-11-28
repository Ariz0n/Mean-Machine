# Запуск T5: одометрия (из T4) + map -> odom + Nav2 + распознавание ArUco
#
# Nav2 поднимает Action-сервер /navigate_to_pose, по цели строит путь и публикует /cmd_vel.

import os

from launch import LaunchDescription
from launch_ros.actions import Node

HERE = os.path.dirname(os.path.abspath(__file__))
ODOM_DIR = os.path.join(os.path.dirname(HERE), 'odom')
PARAMS = os.path.join(HERE, 'nav2_params.yaml')
BT = os.path.join(HERE, 'navigate.xml')

NAV2_NODES = ['controller_server', 'planner_server', 'behavior_server', 'bt_navigator']


def generate_launch_description():
    sim_time = {'use_sim_time': True}
    nodes = [
        # одометрия: /odom и TF odom -> trunk (тот же узел, что в T4)
        # + /scan_cloud: точки скана в системе odom без пола - для карт стоимости Nav2.
        # Если упадет - перезапустится через 1 с (map_odom сохранит позу робота на карте)
        Node(executable=os.path.join(ODOM_DIR, 'odom_node.py'), name='odom_node', output='screen',
             parameters=[{'cloud': True}], respawn=True, respawn_delay=1.0),
        # TF map -> odom
        Node(executable=os.path.join(HERE, 'map_odom.py'), name='map_odom', output='screen'),
        # ArUco -> /aruco/detections; если упадет - перезапустится через 1 с
        Node(executable=os.path.join(HERE, 'aruco_node.py'), name='aruco_node', output='screen',
             respawn=True, respawn_delay=1.0),

        # Nav2
        Node(package='nav2_controller', executable='controller_server', name='controller_server',
             output='screen', parameters=[PARAMS, sim_time]),
        Node(package='nav2_planner', executable='planner_server', name='planner_server',
             output='screen', parameters=[PARAMS, sim_time]),
        Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server',
             output='screen', parameters=[PARAMS, sim_time]),
        # цель принимаем только через Action /navigate_to_pose (так по ТЗ). Вспомогательный топик
        # /goal_pose симулятора не слушаем - иначе он перебил бы цель от Judge
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
             output='screen', parameters=[PARAMS, sim_time, {'default_nav_to_pose_bt_xml': BT}],
             remappings=[('goal_pose', 'bt_navigator/goal_pose_unused')]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_navigation',
             output='screen', parameters=[sim_time, {'autostart': True, 'node_names': NAV2_NODES,
                                                      'bond_timeout': 0.0}]),
    ]
    return LaunchDescription(nodes)
