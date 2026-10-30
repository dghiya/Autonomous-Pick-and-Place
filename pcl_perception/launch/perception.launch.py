import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from moveit_configs_utils import MoveItConfigsBuilder # <-- 1. Added Import


def generate_launch_description():
    pkg_share = get_package_share_directory('pcl_perception')
    world_path = os.path.join(pkg_share, 'worlds', 'pcl_scene.sdf')
    urdf_path = os.path.join(pkg_share, 'urdf', 'ur5e_robotiq.urdf.xacro')

    moveit_pkg_share = get_package_share_directory('ur5e_gripper_moveit_config')
    controllers_path = os.path.join(moveit_pkg_share, 'config', 'ros2_controllers.yaml')

    robotiq_share = os.path.join(get_package_prefix('robotiq_description'), 'share')
    set_env_var = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', robotiq_share
    )
    
    rviz_config_file = os.path.join(
        get_package_share_directory('ur5e_gripper_moveit_config'),
        'config',
        'moveit.rviz'
    )
    
    ur5e_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ur_simulation_gz'), 'launch', 'ur_sim_control.launch.py')
        ),
        launch_arguments={
            'ur_type': 'ur5e',
            'world_file': world_path,
            'description_file': urdf_path,
            'controllers_file': controllers_path,
            'initial_joint_controller': 'ur_manipulator_controller',
            'launch_rviz': 'false',
        }.items()
    )

    # --- 2. NEW MOVEIT CONFIGURATION & TIME SYNC ---
    # Build MoveIt config forcing it to use your Gazebo URDF
    moveit_config = (
        MoveItConfigsBuilder("ur5e_robotiq", package_name="ur5e_gripper_moveit_config")
        .robot_description(file_path=urdf_path)
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )
    # Merge use_sim_time into the MoveIt dictionary
    moveit_params = moveit_config.to_dict()
    moveit_params.update({'use_sim_time': True})

    # Launch MoveGroup directly using the merged dictionary
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        output="screen",
        parameters=[moveit_params],
    )
    # -----------------------------------------------

    gripper_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "-c", "/controller_manager"],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/camera_fl/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/camera_fr/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/camera_bl/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/camera_br/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked'
        ]
    )

    tf_fl = Node(package='tf2_ros', executable='static_transform_publisher',
                 arguments=['--x', '0.1', '--y', '0.5', '--z', '1.2',
                            '--roll', '0.0', '--pitch', '0.785', '--yaw', '-0.785',
                            '--frame-id', 'world', '--child-frame-id', 'realsense_front_left/camera_link_fl/camera_fl'])
    tf_fr = Node(package='tf2_ros', executable='static_transform_publisher',
                 arguments=['--x', '0.1', '--y', '-0.5', '--z', '1.2',
                            '--roll', '0.0', '--pitch', '0.785', '--yaw', '0.785',
                            '--frame-id', 'world', '--child-frame-id', 'realsense_front_right/camera_link_fr/camera_fr'])
    tf_bl = Node(package='tf2_ros', executable='static_transform_publisher',
                 arguments=['--x', '1.1', '--y', '0.5', '--z', '1.2',
                            '--roll', '0.0', '--pitch', '0.785', '--yaw', '-2.356',
                            '--frame-id', 'world', '--child-frame-id', 'realsense_back_left/camera_link_bl/camera_bl'])
    tf_br = Node(package='tf2_ros', executable='static_transform_publisher',
                 arguments=['--x', '1.1', '--y', '-0.5', '--z', '1.2',
                            '--roll', '0.0', '--pitch', '0.785', '--yaw', '2.356',
                            '--frame-id', 'world', '--child-frame-id', 'realsense_back_right/camera_link_br/camera_br'])

    perception_node = Node(
        package='pcl_perception',
        executable='pcl_perception_node',
        name='pcl_perception_node',
        output='screen'
    )
    
    # --- 3. FIX RVIZ CRASH ---
    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     arguments=['-d', rviz_config_file],
    #     parameters=[moveit_params], # Sync RViz using the same merged parameters
    #     output='screen'
    # )
    
    foxglove = IncludeLaunchDescription(
    AnyLaunchDescriptionSource(
        os.path.join(get_package_share_directory('foxglove_bridge'), 'launch', 'foxglove_bridge_launch.xml')
    ),
    launch_arguments={
        'port': '8765',
        'send_buffer_limit': '100000000',
        'use_sim_time': 'true',
    }.items()
)

    return LaunchDescription([
        set_env_var,
        ur5e_control,
        move_group_node, # --- 4. Replaced the IncludeLaunchDescription with the direct Node ---
        gripper_spawner,
        bridge,
        tf_fl, tf_fr, tf_bl, tf_br,
        perception_node,
        # rviz_node
        foxglove
    ])