import os
import sys
import xacro
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter, LoadComposableNodes, ComposableNodeContainer
from ament_index_python.packages import get_package_share_directory

from launch.actions import IncludeLaunchDescription

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

def launch_setup(context, *args, **kwargs):
    robot_make = LaunchConfiguration('robot_make').perform(context)
    robot_model = LaunchConfiguration('robot_model').perform(context)
    workstation = LaunchConfiguration('workstation').perform(context)
    robot_source = LaunchConfiguration('robot_source').perform(context)

    description_package = f"{robot_source}_description"
    bringup_package = f"{robot_source}_bringup"
    moveit_config_package = f"{robot_model}_moveit_config"
    gazebo_package = f"{robot_source}_gazebo"

    robot_description_pkg = get_package_share_directory(description_package)
    bringup_package_path = get_package_share_directory(bringup_package)
    moveit_config_path = get_package_share_directory(moveit_config_package)
    gazebo_package_path = get_package_share_directory(gazebo_package)
    ros_gz_sim_path = get_package_share_directory('ros_gz_sim')

    # Robot Description
    xacro_path = os.path.join(robot_description_pkg, f'{robot_model}', 'xacro', f'{robot_model}' + (f'_{workstation}' if workstation else '') + '.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_path)
    robot_description = {'robot_description': robot_description_config.toxml()}

    # SRDF
    robot_description_semantic_config = load_file(moveit_config_package, f'config/{robot_model}.srdf')
    robot_description_semantic = {'robot_description_semantic': robot_description_semantic_config}

    # Kinematics
    kinematics_yaml = load_yaml(moveit_config_package, 'config/kinematics.yaml')
    robot_description_kinematics = {'robot_description_kinematics': kinematics_yaml}

    # Planning Group
    planning_group = f"{robot_make}_arm"

    # OMPL Planning
    ompl_planning_pipeline_config = {
        'planning_pipelines': ['ompl'],
        'ompl': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': """default_planner_request_adapters/AddTimeOptimalParameterization \
                                    default_planner_request_adapters/FixWorkspaceBounds \
                                    default_planner_request_adapters/FixStartStateBounds \
                                    default_planner_request_adapters/FixStartStateCollision \
                                    default_planner_request_adapters/FixStartStatePathConstraints""",
            'start_state_max_bounds_error': 0.1,
        },
    }
    ompl_planning_yaml = load_yaml(moveit_config_package, 'config/ompl_planning.yaml')
    ompl_planning_pipeline_config['ompl'].update(ompl_planning_yaml)

    # Controllers
    controllers_yaml = load_yaml(moveit_config_package, 'config/controllers.yaml')
    moveit_controllers = {
        'moveit_simple_controller_manager': controllers_yaml,
        'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager'
    }

    # Joint Limits
    joint_limits_yaml = {
        'robot_description_planning': load_yaml(
            moveit_config_package, 'config/joint_limits.yaml')
    }

    # Other params
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

    # Launch gz_sim
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_path, 'launch', 'gz_sim.launch.py')),
        launch_arguments=[
            ('gz_args', f"-r {gazebo_package_path}/{robot_model}/worlds/{robot_model}.sdf --physics-engine gz-physics-bullet-featherstone-plugin")
        ],
    )

    pause_sim = ExecuteProcess(
        cmd=[
            'gz', 'service', '-s', f'/world/{robot_model}/control',
            '--reqtype', 'gz.sim.v8.WorldControl',
            '--reptype', 'gz.sim.v8.Boolean',
            '--timeout', '300',
            '--req', '{"pause": true}'
        ],
        output='screen'
    )

    # Spawn the robot model into the gazebo world
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", robot_model,
            "-topic", "/robot_description",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.25",
        ],
        output="screen",
    )

    spawn_camera = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', f'{gazebo_package_path}/rgbd_camera/model/rgbd_camera_model.sdf',
            '-name', 'rgbd_camera',
            '-x', '0.5', '-y', '0.0', '-z', '2.0', '-R', '0.0', '-P', '1.5708', '-Y', '0.0',  # Adjust pose if needed
        ],
        output='screen'
    )

    sim_camera_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0.5", "0", "1.75", "0", "1.5708", "0", "simple_pedestal", "rgbd_camera/camera_link/rgbd_camera"],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(gazebo_package_path, f'{robot_model}', 'config', f'{robot_model}_bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )

    run_move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        emulate_tty=True,
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            joint_limits_yaml,
            {"use_sim_time": True},
        ],
    )

    rviz_config_file = os.path.join(moveit_config_path, 'launch', 'run_move_group.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        emulate_tty=True,
        arguments=['-d', rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            robot_description_kinematics,
            joint_limits_yaml,
        ],
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[robot_description]
    )

    load_arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[f"{robot_make}_arm_controller"],
        output="screen",
    )

    load_hand_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[f"{robot_make}_hand_controller"],
        output="screen",
    )

    # move_named = Node(
    #     package="robot_common_manip",
    #     executable="move_to_named_pose_service",
    #     name="move_to_named_pose_service",
    #     output="screen",
    #     parameters=[
    #         {"planning_group": planning_group},
    #         robot_description,
    #         robot_description_semantic,
    #     ],
    # )

    # move_pose = Node(
    #     package="robot_common_manip",
    #     executable="move_to_pose_service",
    #     name="move_to_pose_service",
    #     output="screen",
    #     parameters=[
    #         {"planning_group": planning_group},
    #         robot_description,
    #         robot_description_semantic,
    #     ],
    # )

    # fake_grasp = Node(
    #     package="robot_common_manip",
    #     executable="fake_grasp_service",
    #     name="fake_grasp_service",
    #     output="screen",
    #     parameters=[
    #         {"planning_group": planning_group},
    #         robot_description,
    #         robot_description_semantic,
    #     ],
    # )

    # start up all of the nodes
    return [
        gz_sim,
        pause_sim,
        # gazebo_nodes,
        # manipulation_nodes,
        spawn_entity,
        spawn_camera,
        sim_camera_tf,
        bridge,
        rviz_node,
        robot_state_publisher,
        run_move_group_node,
        load_arm_controller,
        load_hand_controller,
        # move_named,
        # move_pose,
        # fake_grasp,
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_make',
            default_value='panda',
            description='Name of the robot make (panda, ur, tmr, etc...) to launch (used to locate packages and files).'
        ),
        DeclareLaunchArgument(
            'robot_model',
            default_value='panda',
            description='Name of the robot model (panda, ur5e, tm5-700, etc...) to launch (used to locate packages and files).'
        ),
        DeclareLaunchArgument(
            'robot_source',
            default_value='armada',
            description='Name of the organization the robot belongs to (Universal Robot, Armada, concept in development...) (used to locate packages and files).'
        ),
        DeclareLaunchArgument(
            'workstation',
            default_value='simple_pedestal',
            description='Name of the pedestal or workstation the robot base is attached to (used to locate packages and files).'
        ),
        OpaqueFunction(function=launch_setup),
    ])