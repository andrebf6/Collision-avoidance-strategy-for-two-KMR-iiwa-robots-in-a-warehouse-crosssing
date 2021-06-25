from launch import LaunchDescription

import launch.actions
import launch_ros.actions


def generate_launch_description():

    return LaunchDescription([
        launch_ros.actions.Node(
            package='kmr_model',
            executable='spawn_kuka.py',
            output='screen',
            arguments=[
                '--robot_sdf', launch.substitutions.LaunchConfiguration('robot_sdf'),
                '--robot_name', launch.substitutions.LaunchConfiguration('robot_name'),
                '--robot_namespace', launch.substitutions.LaunchConfiguration('robot_namespace'),
                '-x', launch.substitutions.LaunchConfiguration('x'),
                '-y', launch.substitutions.LaunchConfiguration('y'),
                '-z', launch.substitutions.LaunchConfiguration('z'),
                '-ox', launch.substitutions.LaunchConfiguration('ox'),
                '-oy', launch.substitutions.LaunchConfiguration('oy'),
                '-oz', launch.substitutions.LaunchConfiguration('oz'),
                '-ow', launch.substitutions.LaunchConfiguration('ow')]),

    ])

