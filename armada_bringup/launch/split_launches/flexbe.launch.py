import os, sys
sys.path.append(os.path.dirname(__file__))

from launch import LaunchDescription
from launch.actions import OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_common import common_launch_arguments, build_context


def launch_setup(context, *args, **kwargs):
    cfg = build_context(
        LaunchConfiguration('robot_make').perform(context),
        LaunchConfiguration('robot_model').perform(context),
        LaunchConfiguration('robot_source').perform(context),
        LaunchConfiguration('workstation').perform(context),
    )

    flexbe_full = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(cfg['flexbe_webui_path'], 'launch', 'flexbe_full.launch.py')
        ),
        launch_arguments={'headless': LaunchConfiguration('headless')}.items(),
    )

    return [flexbe_full]



def generate_launch_description():
    return LaunchDescription([
        *common_launch_arguments(include_flexbe=True),
        OpaqueFunction(function=launch_setup),
    ])
