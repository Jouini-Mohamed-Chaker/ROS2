from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Get the directory of the current file
    current_dir = os.path.dirname(os.path.realpath(__file__))

    return LaunchDescription([
        # Start Gazebo with the specified SDF file
        ExecuteProcess(
            cmd=['ign', 'gazebo', os.path.join(current_dir, '../model/line_world.sdf')],
            output='screen'
        ),
        # Start the line follower node
        Node(
            package='line_bot',
            executable='line_follower_node',
            name='line_follower_node',
            output='screen'
        )
    ])

if __name__ == '__main__':
    generate_launch_description()