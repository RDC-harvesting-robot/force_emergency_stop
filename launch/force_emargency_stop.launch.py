from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    leptrino_dir = get_package_share_directory('leptrino_force_sensor')

    leptrino_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(leptrino_dir, 'launch', 'leptrino_launch.py')
        )
    )

    emergency_stop_node = Node(
        package='force_emergency_stop',
        executable='force_emergency_stop',
        name='force_emergency_stop_node',
        output='screen'
    )

    return LaunchDescription([
        leptrino_launch,
        emergency_stop_node
    ])
