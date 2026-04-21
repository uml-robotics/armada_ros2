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

    cgn_cloud_bringup = Node(
        package='contact_graspnet_ros2',
        executable='grasp_executor_cloud_server',
        name='grasp_executor_cloud_server',
        output='screen',
        condition=IfCondition(LaunchConfiguration('launch_cgn_cloud')),
    )

    cgn_rgbd_bringup = Node(
        package='contact_graspnet_ros2',
        executable='grasp_executor_rgbd_server',
        name='grasp_executor_rgbd_server',
        output='screen',
        condition=IfCondition(LaunchConfiguration('launch_cgn_rgbd')),
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

    graspsam_bringup = Node(
        package='graspsam_ros2',
        executable='graspsam_cam2pose_server.py',
        name='graspsam_cam2pose_server',
        output='screen',
        condition=IfCondition(LaunchConfiguration('launch_graspsam')),
    )

    return [
        cgn_cloud_bringup,
        cgn_rgbd_bringup,
        uoc_cloud_bringup,
        uoc_rgbd_bringup,
        graspsam_bringup,
    ]



def generate_launch_description():
    return LaunchDescription([
        *common_launch_arguments(include_perception_toggles=True),
        OpaqueFunction(function=launch_setup),
    ])
