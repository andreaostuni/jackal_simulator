from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from pathlib import Path

from ament_index_python.packages import get_package_share_directory

ARGUMENTS = [
    DeclareLaunchArgument('world_path', default_value='',
                          description='The world path, by default is empty.world'),
]


def generate_launch_description():

    gz_resource_path = SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=[
        EnvironmentVariable('GAZEBO_MODEL_PATH',
                            default_value=''),
        '/usr/share/gazebo-11/models/:',
        str(Path(get_package_share_directory('jackal_description')).
            parent.resolve())])

    # Launch args
    world_path = LaunchConfiguration('world_path')
    prefix = LaunchConfiguration('prefix')

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
        'is_sim:=true',
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
                   '-x -3.45',
                   '-y 0.05',
                   '-z 0.31',
                   '-Y 0.0'],
        output='screen',
    )

    # Launch jackal_control/control.launch.py
    launch_jackal_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('jackal_control'), 'launch', 'control.launch.py']
        )),
        launch_arguments=[('robot_description_command', robot_description_command),
                          ('is_sim', 'True')]
    )

    spawn_jackal_localization = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node',
        output='screen',
        parameters=[config_jackal_localization]
    )

    spawn_twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        remappings={
            ('/cmd_vel_out', '/jackal_velocity_controller/cmd_vel_unstamped')},
        parameters=[config_twist_mux],
    )

    spawn_jackal_controllers = GroupAction([
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['jackal_velocity_controller',
                       '-c', '/controller_manager'],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
            output='screen',
        )
    ])

    # Launch jackal_control/teleop_base.launch.py which is various ways to tele-op
    # the robot but does not include the joystick. Also, has a twist mux.
    launch_jackal_teleop_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('jackal_control'), 'launch', 'teleop_base.launch.py'])))

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(gz_resource_path)
    ld.add_action(gzserver)
    ld.add_action(gzclient)
    ld.add_action(spawn_jackal_controllers)
    ld.add_action(spawn_jackal_localization)
    ld.add_action(spawn_twist_mux)
    ld.add_action(launch_jackal_description)
    ld.add_action(spawn_robot)
    ld.add_action(launch_jackal_control)
    ld.add_action(launch_jackal_teleop_base)

    return ld
