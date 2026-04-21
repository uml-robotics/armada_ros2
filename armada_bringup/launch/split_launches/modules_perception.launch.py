import os, sys
sys.path.append(os.path.dirname(__file__))

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
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

    uoc_cloud_bringup = Node(
        package='unseen_obj_clst_ros2',
        executable='segmentation_cloud_server',
        name='segmentation_cloud_server',
        output='screen',
        condition=IfCondition(LaunchConfiguration('launch_uoc_cloud')),
    )

    uoc_rgbd_bringup = Node(
        package='unseen_obj_clst_ros2',
        executable='segmentation_rgbd_server',
        name='segmentation_rgbd_server',
        output='screen',
        condition=IfCondition(LaunchConfiguration('launch_uoc_rgbd')),
    )

    return [
        get_pointcloud_service,
        euclidean_clustering_service,
        filter_by_indices_service,
        uoc_cloud_bringup,
        uoc_rgbd_bringup,
    ]


def generate_launch_description():
    return LaunchDescription([
        *common_launch_arguments(include_perception_toggles=True),
        OpaqueFunction(function=launch_setup),
    ])
