import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import TimerAction


def generate_launch_description():

    # ''use_sim_time'' is used to  determine whether ros2 use simulation time provided by simulation environment (Gazebo).
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    urdf_file_name = 'r2d2.urdf.xml'

    # urdf is path of urdf_file_name file. we use define it as the follow due to the CMakeLists.txt file.
    urdf = os.path.join(
        get_package_share_directory('can_bridge_cpp'),
        'urdf',
        urdf_file_name
    )

    # open the whole urdf_file_name file and read it content to robot_desc
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

        return LaunchDescription([
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='false',
                description='Use simulation (Gazebo) clock if true'),
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='log',
                parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
                arguments=[urdf]),
            # Node(
            #     package='joint_state_publisher_gui',
            #     executable='joint_state_publisher_gui',
            #     name='joint_state_publisher_gui',
            #     output='screen'),
            
            Node(
                package='can_bridge_cpp',
                executable='dm4340_can_sender',
                name='dm4340_can_sender',
                output='screen'),
            TimerAction(
            period=5.0,
            actions=[
                Node(
                package='can_bridge_cpp',
                executable='dm4340_can_receiver',
                name='dm4340_can_receiver',
                output='log',
                arguments=['--ros-args', '--log-level', 'ERROR']
                )
            ]),
            Node(
                package='joy',
                executable='joy_node',
                name='joy_node',
                output='log',
                arguments=['--ros-args', '--log-level', 'ERROR']),
            Node(
                package='can_bridge_cpp',
                executable='joy_control',
                name='joy_control',
                output='log',
                arguments=['--ros-args', '--log-level', 'ERROR']),
            # Node(
            # package='can_bridge_cpp',
            # executable='ros2_param',
            # name='ros2_param',
            # output='screen',
            # emulate_tty=True,
            # arguments=['--ros-args', '--log-level', 'ERROR']),
        ])