from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Get the directory of the current file
    current_dir = os.path.dirname(os.path.realpath(__file__))

    return LaunchDescription([
        # Start Gazebo with the specified SDF file
        ExecuteProcess(
            cmd=['ign', 'gazebo', os.path.join(current_dir, '../model/robot.sdf')],
            output='screen'
        ),
        # Start the lidar_node
        Node(
            package='my_bot',
            executable='lidar_node',
            name='lidar_node',
            output='screen'
        ),
        # Start the robot state publisher with the URDF file
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'robot_description': open(os.path.join(current_dir, '../model/robot.urdf')).read()
            }]
        ),
        # Start RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(current_dir, '../config/visualization.rviz')]
        ),
        # Include the bridge launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(current_dir, 'bridge.launch.py'))
        )
    ])

if __name__ == '__main__':
    generate_launch_description()