import os, sys
sys.path.append(os.path.dirname(__file__))
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from launch_common import common_launch_arguments, build_context


def launch_setup(context, *args, **kwargs):
    cfg = build_context(
        LaunchConfiguration('robot_make').perform(context),
        LaunchConfiguration('robot_model').perform(context),
        LaunchConfiguration('robot_source').perform(context),
        LaunchConfiguration('workstation').perform(context),
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', cfg['robot_model'], '-topic', '/robot_description', '-x', '0.0', '-y', '0.0', '-z', '0.25'],
        output='screen',
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(cfg['gazebo_package_path'], cfg['robot_model'], 'config', f"{cfg['robot_model']}_bridge.yaml"),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen',
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[cfg['robot_description']],
    )

    load_arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[f"{cfg['robot_make']}_arm_controller"],
        output='screen',
    )

    load_hand_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[f"{cfg['robot_make']}_hand_controller"],
        output='screen',
    )

    return [
        robot_state_publisher,
        spawn_entity,
        bridge,
        load_arm_controller,
        load_hand_controller,
    ]



def generate_launch_description():
    return LaunchDescription([
        *common_launch_arguments(),
        OpaqueFunction(function=launch_setup),
    ])
