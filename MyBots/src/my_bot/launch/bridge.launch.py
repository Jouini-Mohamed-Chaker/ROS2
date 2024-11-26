from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros_ign_bridge',
            executable='parameter_bridge',
            name='lidar_bridge',
            output='screen',
            arguments=[
                '/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan'
            ]
        )
    ])

if __name__ == '__main__':
    generate_launch_description()