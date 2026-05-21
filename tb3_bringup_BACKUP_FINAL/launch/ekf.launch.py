"""
ekf.launch.py
-------------
Paso 2: Fusión sensorial con robot_localization (EKF).

Lanza:
  - ekf_filter_node  → fusiona /odom + /imu con Kalman Extendido
                       Publica: /odometry/filtered  (odom → base_footprint, más suave)
                       Publica TF: odom → base_footprint (reemplaza al del bridge)

¿Por qué EKF?
  El odómetro solo acumula error. El IMU deriva en ángulo.
  El EKF combina ambas fuentes para una estimación más robusta de la pose.

Cadena TF con EKF activo:
  odom → base_footprint   ← EKF (reemplaza al DiffDrive del bridge)
  base_footprint → base_link → ...  ← robot_state_publisher
  map → odom              ← SLAM o AMCL
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    pkg_bringup = get_package_share_directory('tb3_bringup')

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(pkg_bringup, 'config', 'ekf.yaml'),
            {'use_sim_time': True},
        ],
    )

    return LaunchDescription([ekf_node])
