"""
Start the manipulation skill server for the LWA 4P with MoveIt on one backend.

sim:=mock    Tomas's demo.launch.py: mock hardware, no physics (default).
sim:=gazebo  Tomas's gazebo.launch.py (Gazebo Harmonic).
sim:=isaac   isaac_moveit.launch.py of manipulation_isaac_sim; start Isaac Sim first
             (isaac_sim/run.sh). Its world:= argument passes through.

The backend's move_group gets the MTC execution capability without changes to the backend's
launch file. The skill server copies the URDF and the SRDF from move_group's parameters, so
it plans with exactly the robot model the backend's move_group executes with.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import SetParameter
from moveit_configs_utils import MoveItConfigsBuilder

MTC_CAPABILITY = 'move_group/ExecuteTaskSolutionCapability'
BACKENDS = {
    'mock': ('lwa4p_moveit_config', 'demo.launch.py'),
    'gazebo': ('lwa4p_gazebo', 'gazebo.launch.py'),
    'isaac': ('manipulation_isaac_sim', 'isaac_moveit.launch.py'),
}


def backend_actions(sim: str) -> list:
    """Return the actions that start the backend with an MTC-ready move_group."""
    package, launch_file = BACKENDS[sim]
    source = PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory(package), 'launch', launch_file))
    if sim == 'mock':
        # demo.launch.py declares arguments for its move_group and RViz.
        return [IncludeLaunchDescription(source, launch_arguments={
            'capabilities': MTC_CAPABILITY,
            'use_rviz': LaunchConfiguration('rviz'),
        }.items())]
    # These launch files do not set the parameter for move_group; a parameter set here reaches
    # every node they start, and the other nodes ignore it.
    return [GroupAction([
        SetParameter(name='capabilities', value=MTC_CAPABILITY),
        IncludeLaunchDescription(source),
    ])]


def launch_nodes(context) -> list:
    sim = LaunchConfiguration('sim').perform(context)
    if sim not in BACKENDS:
        raise RuntimeError(f'unknown sim {sim!r}; choose one of: {", ".join(BACKENDS)}')

    moveit_config = MoveItConfigsBuilder(
        'lwa4p', package_name='lwa4p_moveit_config').to_moveit_configs()
    server = Node(
        package='manipulation_skill_server', executable='skill_server_node',
        name='skill_server', output='screen',
        # No robot_description parameters: the server copies them from move_group.
        parameters=[
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            moveit_config.planning_pipelines,
            os.path.join(
                get_package_share_directory('lwa4p_bringup'), 'config', 'skill_server.yaml'),
            {'use_sim_time': sim != 'mock'},
        ])
    return [*backend_actions(sim), server]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='mock',
                              description='backend: ' + ', '.join(BACKENDS)),
        DeclareLaunchArgument('rviz', default_value='true', description='RViz with MoveIt'),
        OpaqueFunction(function=launch_nodes),
    ])
