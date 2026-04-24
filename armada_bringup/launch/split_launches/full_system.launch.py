import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


THIS_DIR = os.path.dirname(__file__)

'''
Example for UOC + CGN RGBD only:
 ros2 launch armada_bringup full_system.launch.py \
   launch_cgn_rgbd:=True launch_uoc_rgbd:=True \
   launch_cgn_cloud:=False launch_uoc_cloud:=False \
   launch_graspsam:=False
'''


def _include(filename, extra_args=None, condition=None):
    args = {
        'robot_make': LaunchConfiguration('robot_make'),
        'robot_model': LaunchConfiguration('robot_model'),
        'robot_source': LaunchConfiguration('robot_source'),
        'workstation': LaunchConfiguration('workstation'),
    }
    if extra_args:
        args.update(extra_args)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(THIS_DIR, filename)),
        launch_arguments=args.items(),
        condition=condition,
    )



def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_make', default_value='panda'),
        DeclareLaunchArgument('robot_model', default_value='panda'),
        DeclareLaunchArgument('robot_source', default_value='armada'),
        # DeclareLaunchArgument('workstation', default_value='simple_pedestal'),
        DeclareLaunchArgument('workstation', default_value='pedestal_workstation'),
        DeclareLaunchArgument('launch_rviz', default_value='True'),
        DeclareLaunchArgument('camera_pose', default_value='tabletop'),
        DeclareLaunchArgument('headless', default_value='False'),
        DeclareLaunchArgument('launch_flexbe', default_value='False'),
        DeclareLaunchArgument('launch_classic_perception', default_value='False'),
        DeclareLaunchArgument('launch_learned_perception', default_value='True'),
        DeclareLaunchArgument('launch_cgn_cloud', default_value='False'),
        DeclareLaunchArgument('launch_cgn_rgbd', default_value='True'),
        DeclareLaunchArgument('launch_uoc_cloud', default_value='False'),
        DeclareLaunchArgument('launch_uoc_rgbd', default_value='True'),
        DeclareLaunchArgument('launch_graspsam', default_value='False'),
        _include('sim_world.launch.py'),
        _include('sim_robot.launch.py'),
        _include('sim_camera.launch.py', {'camera_pose': LaunchConfiguration('camera_pose')}),
        _include('moveit_core.launch.py', {'launch_rviz': LaunchConfiguration('launch_rviz')}),
        _include('modules_classic.launch.py', condition=IfCondition(LaunchConfiguration('launch_modules_classic'))),
        _include(
            'modules_learned.launch.py',
            {
                'launch_cgn_cloud': LaunchConfiguration('launch_cgn_cloud'),
                'launch_cgn_rgbd': LaunchConfiguration('launch_cgn_rgbd'),
                'launch_uoc_cloud': LaunchConfiguration('launch_uoc_cloud'),
                'launch_uoc_rgbd': LaunchConfiguration('launch_uoc_rgbd'),
                'launch_graspsam': LaunchConfiguration('launch_graspsam'),
            },
            condition=IfCondition(LaunchConfiguration('launch_modules_learned')),
        ),
        _include('flexbe.launch.py', {'headless': LaunchConfiguration('headless')}, condition=IfCondition(LaunchConfiguration('launch_flexbe'))),
    ])
