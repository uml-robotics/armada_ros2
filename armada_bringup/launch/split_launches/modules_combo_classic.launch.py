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

    detect_grasps = Node(
        package='gpd_ros',
        executable='grasp_detection_server',
        name='grasp_detection_server',
        output='screen',
        parameters=[
            {'camera_position': [0.0, 0.0, 0.0]},
            {'config_file': os.path.expanduser('~/flexbe_ws/gpd/cfg/ros_eigen_params.cfg')},
            {'grasps_topic': 'clustered_grasps'},
            {'service_name': 'detect_grasps'},
        ],
    )

    compute_grasp_poses = Node(
        package='gpd_ros',
        executable='grasp_pose_server',
        name='grasp_pose_server',
        output='screen',
        parameters=[
            {'gripper_offset': 0.0},
            {'approach_dist': 0.10},
            {'retreat_dist': 0.0},
            {'grasp_rot_x': 0.0},
            {'grasp_rot_y': 0.0},
            {'grasp_rot_z': 0.0},
            {'grasp_rot_w': 1.0},
            {'target_frame': f"{cfg['robot_make']}_link0" if cfg['robot_make'] != 'panda' else 'panda_link0'},
            {'source_frame': cfg['workstation']},
        ],
    )

    base_service_params = [
        {'default_camera_topic': '/camera/depth/points'},
        {'target_frame': f"{cfg['robot_make']}_link0" if cfg['robot_make'] != 'panda' else 'panda_link0'},
        {'timeout_sec': 3.0},
    ]

    get_pointcloud_service = Node(
        package='compare_flexbe_utilities',
        executable='get_point_cloud_service',
        name='get_point_cloud_service',
        output='screen',
        parameters=base_service_params,
    )

    euclidean_clustering_service = Node(
        package='compare_flexbe_utilities',
        executable='euclidean_clustering_service',
        name='euclidean_clustering_service',
        output='screen',
        parameters=base_service_params,
    )

    filter_by_indices_service = Node(
        package='compare_flexbe_utilities',
        executable='filter_by_indices_service',
        name='filter_by_indices_service',
        output='screen',
        parameters=base_service_params,
    )

    return [
        detect_grasps,
        compute_grasp_poses,
        get_pointcloud_service,
        euclidean_clustering_service,
        filter_by_indices_service,
    ]



def generate_launch_description():
    return LaunchDescription([
        *common_launch_arguments(),
        OpaqueFunction(function=launch_setup),
    ])
