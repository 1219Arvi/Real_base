import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import  RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import Command
from launch.actions import TimerAction
from launch.event_handlers import OnProcessStart

def generate_launch_description():
    package_name = 'bot' 
    bot_share = get_package_share_directory(package_name)
    desc_share= get_package_share_directory("robot_description")

    xacro_file = os.path.join(desc_share, 'urdf', 'robot.xacro')
    doc = xacro.process_file(xacro_file)
    robot_description = {'robot_description': doc.toxml()}

    controller_params_file = os.path.join(bot_share, 'config', 'diff_drive_controller.yaml')


    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description, {'use_sim_time': False}],
        output='screen',
        arguments=['--ros-args', '--log-level', 'debug']
        
    )
    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_description},controller_params_file],
        output='screen',
        arguments=['--ros-args', '--log-level', 'debug']

    )

    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])

    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    load_diff_drive_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_base_controller',],
        output='screen'
    )

    delayed_diff_drive_base_controller = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[load_diff_drive_controller],
        )
    )

    delayed_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[load_joint_state_broadcaster],
        )
    )

    return LaunchDescription([
        delayed_controller_manager,
        robot_state_publisher,
        delayed_diff_drive_base_controller,
        delayed_joint_state_broadcaster,
 
    ])
