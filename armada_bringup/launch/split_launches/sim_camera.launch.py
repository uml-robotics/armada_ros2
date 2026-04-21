import os, sys
sys.path.append(os.path.dirname(__file__))

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from launch_common import common_launch_arguments, build_context


CAMERA_PRESETS = {
    'topdown': {
        'spawn': ['-x', '0.5', '-y', '0.0', '-z', '1.5', '-R', '0.0', '-P', '1.5708', '-Y', '0.0'],
        'tf': ['0.5', '0', '1.25', '0', '1.5708', '0'],
    },
    'angled45': {
        'spawn': ['-x', '0.0', '-y', '0.0', '-z', '1.0', '-R', '0.0', '-P', '0.7854', '-Y', '0.0'],
        'tf': ['0.0', '0', '0.75', '0', '0.7854', '0'],
    },
    'tabletop': {
        'spawn': ['-x', '0.0', '-y', '0.2', '-z', '1.5', '-R', '0.0', '-P', '1.2', '-Y', '0.0'],
        'tf': ['0.0', '0.2', '1.25', '0', '1.2', '0'],
    },
}


def launch_setup(context, *args, **kwargs):
    cfg = build_context(
        LaunchConfiguration('robot_make').perform(context),
        LaunchConfiguration('robot_model').perform(context),
        LaunchConfiguration('robot_source').perform(context),
        LaunchConfiguration('workstation').perform(context),
    )
    camera_pose = LaunchConfiguration('camera_pose').perform(context)
    preset = CAMERA_PRESETS.get(camera_pose, CAMERA_PRESETS['tabletop'])

    spawn_camera = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', os.path.join(cfg['gazebo_package_path'], 'rgbd_camera', 'model', 'rgbd_camera_model.sdf'),
            '-name', 'rgbd_camera',
            *preset['spawn'],
        ],
        output='screen',
    )

    

    sim_camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        # arguments=[*preset['tf'], cfg['workstation'], 'rgbd_camera/camera_link/rgbd_camera'],
        arguments=[*preset['tf'],'simple_pedestal','rgbd_camera/camera_link/rgbd_camera'], # or just set as ""
        output='screen',
    )

    return [spawn_camera, sim_camera_tf]



def generate_launch_description():
    return LaunchDescription([
        *common_launch_arguments(include_camera_pose=True),
        OpaqueFunction(function=launch_setup),
    ])
