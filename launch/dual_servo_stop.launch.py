from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():

    # パラメータ（必要なら外から値を上書きできるようにしておく）
    threshold_arg = DeclareLaunchArgument(
        'threshold',
        default_value='5.5',
        description='Force threshold to trigger cancel'
    )

    # 左アーム用のノード
    left = GroupAction([
        PushRosNamespace('left_arm'),
        Node(
            package='force_emergency_stop', 
            executable='servo_stop',  
            name='trajectory_cancel',
            output='screen',
            parameters=[{'threshold': LaunchConfiguration('threshold')}],
            remappings=[#名前空間を再定義
                ('/scaled_joint_trajectory_controller/follow_joint_trajectory',
                 '/left_arm/servo_node/delta_twist_cmds'),
                ('/calibrated_force_data',
                 '/left/calibrated_force_data'),
                ('/servo_node/delta_twist_cmds_filtered',
                 '/left_arm/servo_node/delta_twist_cmds_filtered')
            ]
        )
    ])

    # 右アーム用のノード
    right = GroupAction([
        PushRosNamespace('right_arm'),
        Node(
            package='force_emergency_stop',
            executable='servo_stop',
            name='trajectory_cancel',
            output='screen',
            parameters=[{'threshold': LaunchConfiguration('threshold')}],
            remappings=[
                ('/scaled_joint_trajectory_controller/follow_joint_trajectory',
                 '/right_arm/servo_node/delta_twist_cmds'),
                ('/calibrated_force_data',
                 '/right/calibrated_force_data'),
                ('/servo_node/delta_twist_cmds_filtered',
                 '/right_arm/servo_node/delta_twist_cmds_filtered')
            ]
        )
    ])

    return LaunchDescription([
        threshold_arg,
        left,
        right
    ])  
