from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    main_node = Node(
        package='slam_main',
        executable='slam_main',
        name='slam_main',
        parameters=[{
            'laser_topics': 'laser1 laser2',
            'pcd_topics': '',
        }]
    )

    ld.add_action(main_node)

    return ld
