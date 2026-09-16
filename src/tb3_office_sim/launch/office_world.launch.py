# Copyright 2026, kibo. Apache-2.0
#
# office_world.launch.py
# ---------------------------------------------------------------------------
# Brings up the 4 x 8 m office arena in Gazebo Harmonic (gz-sim 8) and spawns
# a TurtleBot3 Burger whose stock URDF has been ported to gz-sim:
#
#   1. Load the vendor URDF (turtlebot3_gazebo::turtlebot3_burger.urdf),
#      rewrite package:// mesh URIs to absolute file:// URIs, and append
#      gz-sim <gazebo> blocks (DiffDrive + gpu_lidar). Classic Gazebo plugins
#      are NOT used anywhere - this is the Harmonic-native port.
#
#   2. robot_state_publisher publishes the static TF tree (base_footprint,
#      base_link, base_scan, wheels) from that URDF.
#
#   3. gz-sim runs office_arena.sdf; ros_gz_sim `create` injects the robot
#      (URDF from /robot_description) at [0.0, 0.0, 0.1].
#
#   4. ros_gz_bridge parameter_bridge relays /clock, /cmd_vel, /odom, /scan
#      between the gz transport bus and ROS 2.
#
#   5. odom_to_tf broadcasts odom -> base_footprint from the /odom message,
#      completing the TF chain that slam_toolbox consumes.
#
#   6. scan_republisher re-frames /scan_raw to 'base_scan' because gz-sim
#      flattens the URDF's fixed joints when converting to SDF, so the raw
#      lidar frame is the sensor's parent link instead of base_scan.
#
# Notes on /tf: classic Gazebo injected odometry TF inside the simulator.
# gz-sim has no such bridge on the ROS side, and its PosePublisher emits
# WORLD-frame poses (which would create a second parent for base_link). The
# correct SLAM topology is: static robot TF from robot_state_publisher +
# dynamic odom -> base_footprint from odom_to_tf. So /tf is generated on the
# ROS side and deliberately not bridged.
#
# Usage:
#   ros2 launch tb3_office_sim office_world.launch.py headless:=true
#   ros2 launch tb3_office_sim office_world.launch.py   # with gz GUI

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def _load_robot_description() -> str:
    """Vendor TB3 burger URDF + gz-sim plugin blocks (Harmonic port)."""
    tb3_share = get_package_share_directory('turtlebot3_gazebo')
    urdf_path = os.path.join(tb3_share, 'urdf', 'turtlebot3_burger.urdf')
    with open(urdf_path, 'r') as f:
        urdf = f.read()

    # package:// URIs are not portable to gz-sim's URDF parser; make them
    # absolute so mesh visuals/collisions resolve on any install tree.
    urdf = urdf.replace('package://turtlebot3_gazebo/',
                        'file://' + tb3_share + '/')

    # The vendor caster is a FIXED box skid (30x9x20 mm) that digs ~3 mm
    # into the floor. With high floor friction it acts as a brake, making
    # the driven wheels slip: odom then over-reports motion by up to ~1.8x
    # and SLAM smears. Swap it for a rolling ball caster (10 mm sphere,
    # bottom exactly on the ground) like the real Burger.
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


def _launch_gz_sim(context, *args, **kwargs):
    """Include gz_sim.launch.py once, adding -s (headless) when requested."""
    world = os.path.join(
        get_package_share_directory('tb3_office_sim'),
        'worlds', 'office_arena.sdf')
    headless = LaunchConfiguration('headless').perform(context)
    gz_args = '-r ' + world + (' -s' if headless == 'true' else '')
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'),
                         'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': gz_args}.items(),
    )
    return [gz_sim]


def generate_launch_description():
    pkg_share = get_package_share_directory('tb3_office_sim')
    robot_description = _load_robot_description()

    # --- Gazebo Harmonic (headless option) ------------------------------
    gz_sim = OpaqueFunction(function=_launch_gz_sim)

    # --- Static TF from the ported URDF ---------------------------------
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': True}],
    )

    # --- Spawn the burger via /world/office_arena/create -----------------
    spawn_tb3 = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_turtlebot3',
        output='screen',
        parameters=[{
            'world': 'office_arena',
            'name': 'turtlebot3_burger',
            'topic': '/robot_description',
            'x': 0.0,
            'y': 0.0,
            'z': 0.1,
            'Y': 0.0,
        }],
    )

    # --- Bridges: gz transport <-> ROS 2 ---------------------------------
    # direction chars: [ = gz->ROS, ] = ROS->gz, @ = both
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
            # gz-sim publishes the lidar on both /model/<name>/scan and /scan;
            # the root /scan carries the actual (tested) data stream.
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        ],
        remappings=[
            ('/model/turtlebot3_burger/cmd_vel', '/cmd_vel'),
            ('/model/turtlebot3_burger/odometry', '/odom'),
            ('/scan', '/scan_raw'),
        ],
        output='screen',
    )

    # --- Re-frame the scan to base_scan (fixed joints are flattened) -----
    scan_republisher = Node(
        package='tb3_office_sim',
        executable='scan_republisher',
        name='scan_republisher',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # --- odom -> base_footprint TF (ROS side) ----------------------------
    odom_tf = Node(
        package='tb3_office_sim',
        executable='odom_to_tf',
        name='odom_to_tf',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='Run gz-sim without the GUI (server only).'),
        gz_sim,
        robot_state_publisher,
        spawn_tb3,
        bridge,
        scan_republisher,
        odom_tf,
    ])