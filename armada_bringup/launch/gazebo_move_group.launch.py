import os
import sys
import xacro
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import PythonExpression, LaunchConfiguration
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
    flexbe_onboard_path = get_package_share_directory('flexbe_onboard')
    flexbe_webui_path = get_package_share_directory('flexbe_webui')

    gazebo_models_path = os.path.join(gazebo_package_path, 'models') 

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

    # Launch FlexBE operator control system (OCS)
    flexbe_full = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(flexbe_webui_path, 'launch', 'flexbe_full.launch.py')),
        launch_arguments={
            'headless': LaunchConfiguration('headless')
        }.items(),
        condition=IfCondition(LaunchConfiguration('launch_flexbe')),
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
            '-x', '0.5', '-y', '0.0', '-z', '2.0', '-R', '0.0', '-P', '1.5708', '-Y', '0.0',  # Adjust pose if needed, camera has no collision
        ],
        output='screen'
    )

    sim_camera_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0.5", "0", "1.75", "0", "1.5708", "0", "simple_pedestal", "rgbd_camera/camera_link/rgbd_camera"],   # transform from base of robot is camera height - height of base over ground (1.5 - 0.25)
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

    move_cartesian = Node(
        package="compare_flexbe_utilities",
        executable="cartesian_move_to_pose_service",
        name="cartesian_move_to_pose_service",
        output="screen",
        parameters=[
            {"planning_group": planning_group},
            robot_description,
            robot_description_semantic,
        ],
    )

    move_pose = Node(
        package="compare_flexbe_utilities",
        executable="move_to_pose_service",
        name="move_to_pose_service",
        output="screen",
        parameters=[
            {"planning_group": planning_group},
            robot_description,
            robot_description_semantic,
        ],
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

    detect_grasps = Node(
        package="gpd_ros",
        executable="grasp_detection_server",
        name="grasp_detection_server",
        output="screen",
        parameters=[
            {"camera_position": [0.0, 0.0, 0.0]},
            {"config_file": '/home/brian/flexbe_ws/gpd/cfg/ros_eigen_params.cfg'},
            {"rviz_topic": "grasp_markers"},  # /rviz_grasps
            {"grasps_topic": "/clustered_grasps"},
        ],
    )

    compute_grasp_poses = Node(
        package="gpd_ros",
        executable="grasp_pose_server",
        name="grasp_pose_server",
        output="screen",
        parameters=[
            {"gripper_offset": -0.02},
            {"approach_dist": 0.0},
            {"retreat_dist": 0.0},
            {"grasp_rot_x": 0.0},
            {"grasp_rot_y": 0.39269908169},
            {"grasp_rot_z": -0.981747704},
            {"grasp_rot_w": 1.0},
            {"target_frame": "panda_link0"},
            {"source_frame": "rgbd_camera/camera_link/rgbd_camera"},
        ],
    )

    get_pointcloud_service = Node(
        package="compare_flexbe_utilities",
        executable="get_point_cloud_service",
        name="get_point_cloud_service",
        output="screen",
        parameters=[
            {"default_camera_topic": "/camera/depth/points"},
            {"target_frame": "base_link"},
            {"timeout_sec": 3.0},
        ],
    )

    euclidean_clustering_service = Node(
        package="compare_flexbe_utilities",
        executable="euclidean_clustering_service",
        name="euclidean_clustering_service",
        output="screen",
        parameters=[
            {"default_camera_topic": "/camera/depth/points"},
            {"target_frame": "base_link"},
            {"timeout_sec": 3.0},
        ],
    )

    filter_by_indices_service = Node(
        package="compare_flexbe_utilities",
        executable="filter_by_indices_service",
        name="filter_by_indices_service",
        output="screen",
        parameters=[
            {"default_camera_topic": "/camera/depth/points"},
            {"target_frame": "base_link"},
            {"timeout_sec": 3.0},
        ],
    )

    spawn_object_1 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', '/home/csrobot/gazebo_models/wood_cube_10cm/model.sdf',
            '-name', 'object_1',
            '-x', '0.55', '-y', '0.05', '-z', '0.65', '-R', '0.0', '-P', '0.0', '-Y', '0.0',  # Adjust pose if needed
        ],
        output='screen'
    )

    spawn_object_2 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', '/home/csrobot/gazebo_models/wood_cube_10cm/model.sdf',
            '-name', 'object_2',
            '-x', '0.45', '-y', '0.15', '-z', '0.65', '-R', '0.0', '-P', '0.0', '-Y', '0.0',  # Adjust pose if needed
        ],
        output='screen'
    )

    spawn_object_3 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', '/home/csrobot/gazebo_models/wood_cube_10cm/model.sdf',
            '-name', 'object_3',
            '-x', '0.5', '-y', '0.10', '-z', '0.65', '-R', '0.0', '-P', '0.0', '-Y', '0.0',  # Adjust pose if needed
        ],
        output='screen'
    )

    detect_grasps = Node(
        package="gpd_ros",
        executable="grasp_detection_server",
        name="grasp_detection_server",
        output="screen",
        parameters=[
            {"camera_position": [0.0, 0.0, 0.0]},
            {"config_file": '/home/csrobot/gpd_ws/gpd/cfg/ros_eigen_params.cfg'},
            {"grasps_topic": 'clustered_grasps'},
            # {"rviz_topic": "grasp_plotter"},
            {"service_name": 'detect_grasps'},
        ],
    )

    gz_services_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge_services',
        # Use CLI-style mappings as arguments (most robust across distros)
        arguments=[
            '/world/panda/create@ros_gz_interfaces/srv/SpawnEntity',
            '/world/panda/remove@ros_gz_interfaces/srv/DeleteEntity',
            '/world/panda/set_pose@ros_gz_interfaces/srv/SetEntityPose',
        ],
        output='screen'
    )

    # start up all of the nodes
    return [
        gz_sim,
        flexbe_full,
        spawn_entity,
        spawn_camera,
        sim_camera_tf,
        bridge,
        rviz_node,
        robot_state_publisher,
        run_move_group_node,
        load_arm_controller,
        load_hand_controller,
        move_cartesian,
        # move_named,
        move_pose,
        get_pointcloud_service,
        euclidean_clustering_service,
        filter_by_indices_service,
        spawn_object_1,
        spawn_object_2,
        spawn_object_3,
        detect_grasps,
        compute_grasp_poses,
        gz_services_bridge,
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
        DeclareLaunchArgument(
            'headless', 
            default_value="False",
            description="Run FlexBE OCS without the web UI frontend."
        ),
        DeclareLaunchArgument(
            'launch_flexbe',
            default_value='False',
            description='If true, start FlexBE (onboard + OCS).'
        ),
        OpaqueFunction(function=launch_setup),
    ])