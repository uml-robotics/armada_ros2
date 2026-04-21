import os, sys
import xacro
import yaml

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except OSError:
        return None



def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except OSError:
        return None



def common_launch_arguments(include_flexbe=False, include_rviz=False, include_camera_pose=False, include_perception_toggles=False):
    args = [
        DeclareLaunchArgument(
            'robot_make',
            default_value='panda',
            description='Robot make used to derive controller/planning names.'
        ),
        DeclareLaunchArgument(
            'robot_model',
            default_value='panda',
            description='Robot model used to locate xacro, MoveIt, and Gazebo config.'
        ),
        DeclareLaunchArgument(
            'robot_source',
            default_value='armada',
            description='Package prefix for description/bringup/gazebo packages.'
        ),
        DeclareLaunchArgument(
            'workstation',
            # default_value='simple_pedestal',
            default_value='pedestal_workstation',
            description='Workstation or pedestal suffix used by the xacro file.'
        ),
    ]

    if include_flexbe:
        args.extend([
            DeclareLaunchArgument(
                'headless',
                default_value='False',
                description='Run FlexBE OCS without the web UI frontend.'
            ),
        ])

    if include_rviz:
        args.extend([
            DeclareLaunchArgument(
                'launch_rviz',
                default_value='False',
                description='If true, launch RViz.'
            ),
        ])

    if include_camera_pose:
        args.extend([
            DeclareLaunchArgument(
                'camera_pose',
                default_value='tabletop',
                description='Camera pose preset: tabletop, topdown, or angled45.'
            ),
        ])

    if include_perception_toggles:
        args.extend([
            DeclareLaunchArgument('launch_cgn_cloud', default_value='False'),
            DeclareLaunchArgument('launch_cgn_rgbd', default_value='True'),
            DeclareLaunchArgument('launch_uoc_cloud', default_value='False'),
            DeclareLaunchArgument('launch_uoc_rgbd', default_value='True'),
            DeclareLaunchArgument('launch_graspsam', default_value='False'),
        ])

    return args



def build_context(robot_make, robot_model, robot_source, workstation):
    description_package = f"{robot_source}_description"
    bringup_package = f"{robot_source}_bringup"
    moveit_config_package = f"{robot_model}_moveit_config"
    gazebo_package = f"{robot_source}_gazebo"

    robot_description_pkg = get_package_share_directory(description_package)
    bringup_package_path = get_package_share_directory(bringup_package)
    moveit_config_path = get_package_share_directory(moveit_config_package)
    gazebo_package_path = get_package_share_directory(gazebo_package)
    ros_gz_sim_path = get_package_share_directory('ros_gz_sim')
    mnet_pkg_path = get_package_share_directory('mnet_scenes_gazebo')
    flexbe_webui_path = get_package_share_directory('flexbe_webui')

    ycb_root = os.path.join(mnet_pkg_path, 'models', 'ycb')

    xacro_path = os.path.join(
        robot_description_pkg,
        robot_model,
        'xacro',
        f"{robot_model}{'_' + workstation if workstation else ''}.urdf.xacro",
    )
    robot_description_config = xacro.process_file(xacro_path)
    robot_description = {'robot_description': robot_description_config.toxml()}

    robot_description_semantic = {
        'robot_description_semantic': load_file(moveit_config_package, f'config/{robot_model}.srdf')
    }
    robot_description_kinematics = {
        'robot_description_kinematics': load_yaml(moveit_config_package, 'config/kinematics.yaml')
    }

    ompl_planning_pipeline_config = {
        'planning_pipelines': ['ompl'],
        'ompl': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': (
                'default_planner_request_adapters/AddTimeOptimalParameterization '
                'default_planner_request_adapters/FixWorkspaceBounds '
                'default_planner_request_adapters/FixStartStateBounds '
                'default_planner_request_adapters/FixStartStateCollision '
                'default_planner_request_adapters/FixStartStatePathConstraints'
            ),
            'start_state_max_bounds_error': 0.5,
        },
    }
    ompl_planning_yaml = load_yaml(moveit_config_package, 'config/ompl_planning.yaml') or {}
    ompl_planning_pipeline_config['ompl'].update(ompl_planning_yaml)

    controllers_yaml = load_yaml(moveit_config_package, 'config/controllers.yaml')
    moveit_controllers = {
        'moveit_simple_controller_manager': controllers_yaml,
        'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
    }

    joint_limits_yaml = {
        'robot_description_planning': load_yaml(moveit_config_package, 'config/joint_limits.yaml')
    }

    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.1,
    }

    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    return {
        'robot_make': robot_make,
        'robot_model': robot_model,
        'robot_source': robot_source,
        'workstation': workstation,
        'description_package': description_package,
        'bringup_package': bringup_package,
        'moveit_config_package': moveit_config_package,
        'gazebo_package': gazebo_package,
        'robot_description_pkg': robot_description_pkg,
        'bringup_package_path': bringup_package_path,
        'moveit_config_path': moveit_config_path,
        'gazebo_package_path': gazebo_package_path,
        'ros_gz_sim_path': ros_gz_sim_path,
        'mnet_pkg_path': mnet_pkg_path,
        'flexbe_webui_path': flexbe_webui_path,
        'ycb_root': ycb_root,
        'robot_description': robot_description,
        'robot_description_semantic': robot_description_semantic,
        'robot_description_kinematics': robot_description_kinematics,
        'ompl_planning_pipeline_config': ompl_planning_pipeline_config,
        'moveit_controllers': moveit_controllers,
        'joint_limits_yaml': joint_limits_yaml,
        'trajectory_execution': trajectory_execution,
        'planning_scene_monitor_parameters': planning_scene_monitor_parameters,
        'planning_group': f'{robot_make}_arm',
    }



def ycb_env_actions(ycb_root):
    return [
        SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=ycb_root),
        SetEnvironmentVariable(name='GZ_SIM_MODEL_PATH', value=ycb_root),
    ]
