import os, sys
sys.path.append(os.path.dirname(__file__))

from launch import LaunchDescription
from launch.actions import OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from launch_common import common_launch_arguments, build_context, ycb_env_actions


def launch_setup(context, *args, **kwargs):
    cfg = build_context(
        LaunchConfiguration('robot_make').perform(context),
        LaunchConfiguration('robot_model').perform(context),
        LaunchConfiguration('robot_source').perform(context),
        LaunchConfiguration('workstation').perform(context),
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(cfg['ros_gz_sim_path'], 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments=[
            ('gz_args', f"-r {cfg['gazebo_package_path']}/{cfg['robot_model']}/worlds/{cfg['robot_model']}.sdf --physics-engine gz-physics-bullet-featherstone-plugin")
        ],
    )

    mnet_spawn_scene = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(cfg['mnet_pkg_path'], 'launch', 'create_scene.launch.py')
        ),
    )

    gz_services_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge_services',
        arguments=[
            f"/world/{cfg['robot_model']}/create@ros_gz_interfaces/srv/SpawnEntity",
            f"/world/{cfg['robot_model']}/remove@ros_gz_interfaces/srv/DeleteEntity",
            f"/world/{cfg['robot_model']}/set_pose@ros_gz_interfaces/srv/SetEntityPose",
        ],
        output='screen',
    )

    return [
        *ycb_env_actions(cfg['ycb_root']),
        gz_sim,
        mnet_spawn_scene,
        gz_services_bridge,
    ]



def generate_launch_description():
    return LaunchDescription([
        *common_launch_arguments(),
        OpaqueFunction(function=launch_setup),
    ])
