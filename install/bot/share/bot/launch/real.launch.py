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
        output='screen'
    )
    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_description},
                    controller_params_file]
    )

    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])

    load_diff_drive_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'diff_drive_base_controller'],
        output='screen'
    )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )

    on_spawn_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=controller_manager,
            on_exit=[load_joint_state_broadcaster]
        )
    )

    on_joint_state_loaded = RegisterEventHandler(
        OnProcessExit(
            target_action=load_joint_state_broadcaster,
            on_exit=[load_diff_drive_controller]
        )
    )

    return LaunchDescription([
        delayed_controller_manager,
        robot_state_publisher,
        on_spawn_exit,
        on_joint_state_loaded,
 
    ])
