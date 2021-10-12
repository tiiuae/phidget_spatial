
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

from os import environ as env


def generate_launch_description():

    pkg_name = "phidget_spatial"
    pkg_share_path = get_package_share_directory(pkg_name)

    return LaunchDescription([
        Node(
            package='phidget_spatial',
            executable='phidget_spatial_node',
            name='phidget_spatial_node',
            namespace=(env.get("DRONE_DEVICE_ID", env.get("USER"))),
            parameters=[ #{"publish_interval_ms": 1000}
                "/media/veracrypt1/source/phidget_spatial/src/phidget_spatial/config/phidget_spatial.yaml"
            ]
        )
    ])
