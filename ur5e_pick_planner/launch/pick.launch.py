import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # Load the MoveIt configuration 
    moveit_config = (
        MoveItConfigsBuilder("ur5e_robotiq", package_name="ur5e_gripper_moveit_config")
        .to_moveit_configs()
    )

    # Sync to Gazebo time
    node_params = moveit_config.to_dict()
    node_params.update({'use_sim_time': True})

    # Launch the C++ node and inject ALL MoveIt parameters (including controllers)
    pick_node = Node(
        package='ur5e_pick_planner',
        executable='pick_node',
        name='pick_planner_node',
        output='screen',
        parameters=[node_params]
    )

    return LaunchDescription([
        pick_node
    ])