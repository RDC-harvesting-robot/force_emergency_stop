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
        PushRosNamespace('left'),
        Node(
            package='force_emergency_stop', 
            executable='force_emergency_stop',  
            name='trajectory_cancel',
            output='screen',
            parameters=[{'threshold': LaunchConfiguration('threshold')}],
            remappings=[#ここで名前空間を書き換えることで左右を切り分ける
                ('/scaled_joint_trajectory_controller/follow_joint_trajectory',
                 '/left/scaled_joint_trajectory_controller/follow_joint_trajectory'),
                ('/calibrated_force_data',
                 '/left/calibrated_force_data'),
                ('/cancel_trajectory',
                 '/left/cancel_trajectory')
            ]
        )
    ])

    # 右アーム用のノード
    right = GroupAction([
        PushRosNamespace('right'),
        Node(
            package='force_emergency_stop',
            executable='force_emergency_stop',
            name='trajectory_cancel',
            output='screen',
            parameters=[{'threshold': LaunchConfiguration('threshold')}],
            remappings=[
                ('/scaled_joint_trajectory_controller/follow_joint_trajectory',
                 '/right/scaled_joint_trajectory_controller/follow_joint_trajectory'),
                ('/calibrated_force_data',
                 '/right/calibrated_force_data'),
                ('/cancel_trajectory',
                 '/right/cancel_trajectory')
            ]
        )
    ])

    return LaunchDescription([
        threshold_arg,
        left,
        right
    ])
