# Copyright 2026, kibo. Apache-2.0
#
# autonomous_mapping.launch.py  (v2 - arena-generic autonomous mapping)
# ---------------------------------------------------------------------------
# One launch that runs the whole v2 pipeline using Nav2 instead of the v1
# hardcoded 26-leg route:
#
#   gz-sim (ANY arena .sdf) + TB3 burger  ->  slam_toolbox (/map)
#   -> Nav2 (planner/controller/costmaps/BT) -> collision monitor
#   -> frontier_explorer (greedy frontier goals via NavigateToPose)
#
# Arena swap: pass a different world file and the same stack maps it. The
# world name is read from the SDF itself, the robot spawn pose is an argument,
# and the frontier explorer has no route table - it reads /map live.
#
# Usage:
#   headless auto-mapping of the office:
#     ros2 launch tb3_office_sim autonomous_mapping.launch.py \
#       world_file:=office_arena.sdf arena_size:="4 8"
#   maze (proves the swap works):
#     ros2 launch tb3_office_sim autonomous_mapping.launch.py \
#       world_file:=maze_arena.sdf arena_size:="5 5"
#   with GUI + RViz (override the headless defaults):
#     ... headless:=false rviz:=true
#   manual goal driving instead of auto-explore:
#     ... explore:=false          # then send goals from RViz/Nav2 panel
#
# v1 (office_world.launch.py + drive_route.py) is untouched and still works.

import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


# ----------------------------- robot description ---------------------------
def _load_robot_description() -> str:
    """Vendor TB3 burger URDF + gz-sim plugin blocks (identical to v1)."""
    tb3_share = get_package_share_directory('turtlebot3_gazebo')
    urdf_path = os.path.join(tb3_share, 'urdf', 'turtlebot3_burger.urdf')
    with open(urdf_path, 'r') as f:
        urdf = f.read()

    urdf = urdf.replace('package://turtlebot3_gazebo/',
                        'file://' + tb3_share + '/')

    # rolling ball caster instead of the fixed skid (see v1 launch docstring)
    urdf = urdf.replace(
        '<origin xyz="0 0.001 0" rpy="0 0 0"/>\n      <geometry>\n'
        '        <box size="0.030 0.009 0.020"/>\n      </geometry>',
        '<origin xyz="0 0.001 0.004" rpy="0 0 0"/>\n      <geometry>\n'
        '        <sphere radius="0.010"/>\n      </geometry>')

    gz_plugins = """
  <gazebo reference="wheel_left_link">
    <mu1>2.0</mu1>
    <mu2>2.0</mu2>
  </gazebo>
  <gazebo reference="wheel_right_link">
    <mu1>2.0</mu1>
    <mu2>2.0</mu2>
  </gazebo>

  <gazebo>
    <plugin filename="libgz-sim-diff-drive-system.so"
            name="gz::sim::systems::DiffDrive">
      <left_joint>wheel_left_joint</left_joint>
      <right_joint>wheel_right_joint</right_joint>
      <wheel_separation>0.160</wheel_separation>
      <wheel_radius>0.033</wheel_radius>
      <odom_publish_frequency>10</odom_publish_frequency>
    </plugin>
  </gazebo>

  <gazebo reference="base_scan">
    <sensor name="scan" type="gpu_lidar">
      <topic>scan</topic>
      <update_rate>10</update_rate>
      <lidar>
        <scan>
          <horizontal>
            <samples>360</samples>
            <resolution>1</resolution>
            <min_angle>-3.14159</min_angle>
            <max_angle>3.14159</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.12</min>
          <max>3.5</max>
          <resolution>0.01</resolution>
        </range>
      </lidar>
      <alwaysOn>1</alwaysOn>
      <visualize>true</visualize>
    </sensor>
  </gazebo>
"""
    return urdf.replace('</robot>', gz_plugins + '</robot>')


def _sdf_world_name(sdf_path: str) -> str:
    """Read the <world name=...> from the SDF (the arena file decides it)."""
    tree = ET.parse(sdf_path)
    world = tree.getroot().find('world')
    if world is None or not world.get('name'):
        raise ValueError(f'no <world name> found in {sdf_path}')
    return world.get('name')


def _resolve_world(arg: str, pkg_share: str) -> str:
    """world_file arg -> absolute path (name in worlds/ or full path)."""
    if os.path.isabs(arg):
        if os.path.isfile(arg):
            return arg
        raise FileNotFoundError(arg)
    candidate = os.path.join(pkg_share, 'worlds', arg)
    if os.path.isfile(candidate):
        return candidate
    raise FileNotFoundError(f'{arg} (tried {candidate})')


def _launch_sim(context, *args, **kwargs):
    """gz-sim include + robot spawn, resolved against the chosen arena."""
    pkg_share = get_package_share_directory('tb3_office_sim')
    world_path = _resolve_world(
        LaunchConfiguration('world_file').perform(context), pkg_share)
    world_name = _sdf_world_name(world_path)
    headless = LaunchConfiguration('headless').perform(context) == 'true'
    sx = float(LaunchConfiguration('spawn_x').perform(context))
    sy = float(LaunchConfiguration('spawn_y').perform(context))
    syaw = float(LaunchConfiguration('spawn_yaw').perform(context))

    gz_args = f'-r {world_path}' + (' -s' if headless else '')
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'),
                         'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': gz_args}.items(),
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_turtlebot3',
        output='screen',
        parameters=[{
            'world': world_name,
            'name': 'turtlebot3_burger',
            'topic': '/robot_description',
            'x': sx,
            'y': sy,
            'z': 0.1,
            'Y': syaw,
        }],
    )
    return [gz_sim, spawn]


def _build_slam_toolbox():
    """slam_toolbox lifecycle node with autostart (CONFIGURE/ACTIVATE)."""
    pkg_share = get_package_share_directory('tb3_office_sim')
    slam_params = os.path.join(pkg_share, 'config',
                               'mapper_params_online_async.yaml')
    node = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        output='screen',
        parameters=[slam_params, {'use_sim_time': True}],
    )
    configure = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        ))
    activate = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                LogInfo(msg='[autonomous_mapping] slam_toolbox activating'),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(node),
                    transition_id=Transition.TRANSITION_ACTIVATE,
                )),
            ],
        ))
    return node, configure, activate


def generate_launch_description():
    pkg_share = get_package_share_directory('tb3_office_sim')

    # ---- world + spawn (arena-agnostic) --------------------------------
    sim = OpaqueFunction(function=_launch_sim)

    # ---- static TF from the ported URDF ---------------------------------
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': _load_robot_description(),
                     'use_sim_time': True}],
    )

    # ---- gz <-> ROS bridge -----------------------------------------------
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        parameters=[{'qos_overrides./model/turtlebot3_burger.subscriber.reliability':
                     'reliable'}],
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/model/turtlebot3_burger/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/model/turtlebot3_burger/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        ],
        remappings=[
            ('/model/turtlebot3_burger/cmd_vel', '/cmd_vel'),
            ('/model/turtlebot3_burger/odometry', '/odom'),
            ('/scan', '/scan_raw'),
        ],
        output='screen',
    )

    scan_republisher = Node(
        package='tb3_office_sim',
        executable='scan_republisher',
        name='scan_republisher',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    odom_tf = Node(
        package='tb3_office_sim',
        executable='odom_to_tf',
        name='odom_to_tf',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # ---- slam_toolbox (same tuned params as v1) --------------------------
    slam_node, slam_configure, slam_activate = _build_slam_toolbox()

    # ---- Nav2 (slam mode: no amcl/map_server, live /map from slam) -------
    nav2_params = os.path.join(pkg_share, 'config', 'nav2', 'nav2_params.yaml')
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'),
                         'launch', 'bringup_launch.py')),
        launch_arguments={
            'slam': 'True',
            'use_localization': 'False',
            'use_sim_time': 'True',
            'autostart': 'True',
            'use_composition': 'True',
            'params_file': nav2_params,
        }.items(),
    )

    # ---- frontier explorer (delayed so Nav2 has time to activate) --------
    # arena_size (optional) flows through as a launch substitution; the node
    # uses it only for the coverage % estimate, never for planning.
    explorer = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='tb3_office_sim',
                executable='frontier_explorer',
                name='frontier_explorer',
                output='screen',
                parameters=[{'use_sim_time': True,
                             'arena_size': LaunchConfiguration('arena_size')}],
                condition=IfCondition(LaunchConfiguration('explore')),
            )
        ],
    )

    # ---- RViz (v2 view: TF, robot, scan, live map, costmaps, plans) ------
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_share, 'config', 'tb3_nav2.rviz')],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[{'use_sim_time': True}],
    )

    ld = LaunchDescription([
        DeclareLaunchArgument(
            'world_file', default_value='office_arena.sdf',
            description='Arena .sdf: filename in worlds/, or absolute path.'),
        DeclareLaunchArgument(
            'spawn_x', default_value='0.0',
            description='Robot spawn x (must be free in the arena).'),
        DeclareLaunchArgument(
            'spawn_y', default_value='0.0',
            description='Robot spawn y (must be free in the arena).'),
        DeclareLaunchArgument(
            'spawn_yaw', default_value='0.0',
            description='Robot spawn yaw (radians).'),
        DeclareLaunchArgument(
            'arena_size', default_value='',
            description='"<W> <H>" meters of the drivable arena, used only '
                        'for the explorer coverage estimate.'),
        DeclareLaunchArgument(
            'headless', default_value='true',
            description='Run gz-sim without the GUI (server only).'),
        DeclareLaunchArgument(
            'explore', default_value='true',
            description='Run the frontier explorer automatically.'),
        DeclareLaunchArgument(
            'rviz', default_value='false',
            description='Start the pre-configured RViz2 view.'),

        sim,
        robot_state_publisher,
        bridge,
        scan_republisher,
        odom_tf,
        slam_node,
        slam_configure,
        slam_activate,
        nav2,
        explorer,
        rviz,
    ])
    return ld