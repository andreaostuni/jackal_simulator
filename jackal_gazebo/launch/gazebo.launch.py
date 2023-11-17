from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, SetEnvironmentVariable,
                            GroupAction, RegisterEventHandler, Shutdown)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (EnvironmentVariable, FindExecutable,
                                  LaunchConfiguration, PathJoinSubstitution,
                                  Command)
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition, UnlessCondition

from pathlib import Path

from ament_index_python.packages import get_package_share_directory

ARGUMENTS = [
    DeclareLaunchArgument('world_path', default_value='',
                          description='The world path, by default is empty.world'),
    DeclareLaunchArgument('prefix', default_value='',
                          description='The prefix of the world file'),
    DeclareLaunchArgument('use_gazebo_controllers', default_value='True',
                          description='Whether to start the gazebo controllers'),
]


def generate_launch_description():

    # gz_resource_path = SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=[
    #     EnvironmentVariable('GAZEBO_MODEL_PATH',
    #                         default_value=''),
    #     '/usr/share/gazebo-11/models/:',
    #     str(Path(get_package_share_directory('jackal_description')).
    #         parent.resolve())])

    # Launch args
    world_path = LaunchConfiguration('world_path')
    prefix = LaunchConfiguration('prefix')
    use_gazebo_controllers = LaunchConfiguration('use_gazebo_controllers')

    config_jackal_velocity_controller = PathJoinSubstitution(
        [FindPackageShare('jackal_gazebo'), 'config', 'control.yaml']
    )

    config_jackal_localization = PathJoinSubstitution(
        [FindPackageShare('jackal_gazebo'), 'config', 'localization.yaml']
    )

    config_twist_mux = PathJoinSubstitution(
        [FindPackageShare('jackal_gazebo'), 'config', 'twist_mux.yaml']
    )

    # Get URDF via xacro
    robot_description_command = [
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution(
            [FindPackageShare('jackal_description'),
             'urdf', 'jackal.urdf.xacro']
        ),
        ' ',
        'use_gazebo_controllers:=',
        use_gazebo_controllers,
        ' ',
        'gazebo_sim:=True',
        ' ',
        'gazebo_controllers:=',
        config_jackal_velocity_controller,
    ]

    launch_jackal_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('jackal_description'),
                 'launch',
                 'description.launch.py']
            )
        ),
        launch_arguments=[('robot_description_command',
                           robot_description_command)]
    )

    # Gazebo server
    gzserver = ExecuteProcess(
        cmd=['gzserver',
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so',
             '--verbose',
             world_path],
        output='screen',
    )

    # Gazebo client
    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
    )

    # Spawn robot
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_jackal',
        arguments=['-entity',
                   'jackal',
                   '-topic',
                   'robot_description',
                   '-x 0.0',
                   '-y 0.0',
                   '-z 0.0',
                   '-Y 0.0'],
        output='screen',
    )

    # Launch jackal_control/control.launch.py
    launch_jackal_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('jackal_control'), 'launch', 'control.launch.py']
        )),
        launch_arguments=[('robot_description_command', robot_description_command),
                          ('gazebo_sim', 'True'),
                          ('config_jackal_velocity',
                           config_jackal_velocity_controller),
                          ('config_jackal_localization',
                           config_jackal_localization),
                          ],
        condition = IfCondition(use_gazebo_controllers)
    )

    spawn_jackal_controllers = GroupAction([
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['jackal_velocity_controller',
                       '-c', '/controller_manager'],
            output='screen',
            condition=IfCondition(use_gazebo_controllers),
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
            output='screen',
            condition=IfCondition(use_gazebo_controllers),
        )
    ])

    # Make sure spawn_jackal_controllers starts after spawn_robot
    jackal_controllers_spawn_callback = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[spawn_jackal_controllers],
        )
    )

    stop_jackal_cmd = ['ros2 topic pub /stop/cmd_vel geometry_msgs/msg/Twist ',
                       '"{ linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"', ]

    stop_jackal = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[ExecuteProcess(
                cmd=stop_jackal_cmd,
                output='log',
                shell=True,
                on_exit=Shutdown(),
                condition=UnlessCondition(
                    use_gazebo_controllers)
            )],
        )
    )

    # Launch jackal_control/teleop_base.launch.py which is various ways to tele-op
    # the robot but does not include the joystick. Also, has a twist mux.
    launch_jackal_teleop_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('jackal_control'), 'launch', 'teleop_base.launch.py'])),
        launch_arguments=[('config_twist_mux', config_twist_mux)]
    )

    ld = LaunchDescription(ARGUMENTS)
    # ld.add_action(gz_resource_path)
    ld.add_action(gzserver)
    ld.add_action(gzclient)
    ld.add_action(jackal_controllers_spawn_callback)
    ld.add_action(launch_jackal_description)
    ld.add_action(spawn_robot)
    ld.add_action(launch_jackal_control)
    ld.add_action(launch_jackal_teleop_base)
    ld.add_action(stop_jackal)

    return ld
