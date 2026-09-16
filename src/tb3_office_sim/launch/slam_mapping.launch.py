# Copyright 2026, kibo. Apache-2.0
#
# slam_mapping.launch.py
# ---------------------------------------------------------------------------
# Runs online (async) SLAM with slam_toolbox plus a pre-configured RViz2.
# Expected TF tree (published by office_world.launch.py and this launch):
#
#   map --(slam_toolbox)--> odom --(odom_to_tf)--> base_footprint
#       --(robot_state_publisher)--> base_link --(fixed)--> base_scan
#
# The Jazzy slam_toolbox node is lifecycle-managed, so like the vendor
# online_async_launch.py we start it as a LifecycleNode and emit CONFIGURE /
# ACTIVATE transitions (autostart=true, no lifecycle manager).
#
# Usage:
#   ros2 launch tb3_office_sim slam_mapping.launch.py
#   ros2 launch tb3_office_sim slam_mapping.launch.py rviz:=false   # headless
# Then drive with: ros2 run tb3_office_sim drive_route
#        (or:     ros2 run teleop_twist_keyboard teleop_twist_keyboard)

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, RegisterEventHandler
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    pkg_share = get_package_share_directory('tb3_office_sim')

    slam_params = os.path.join(
        pkg_share, 'config', 'mapper_params_online_async.yaml')
    rviz_config = os.path.join(pkg_share, 'config', 'tb3_slam.rviz')

    slam_toolbox = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        output='screen',
        parameters=[slam_params, {'use_sim_time': True}],
    )

    # autostart: CONFIGURE then ACTIVATE once the node reaches "inactive"
    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_toolbox),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )
    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_toolbox,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                LogInfo(msg='[slam_mapping] slam_toolbox activating.'),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(slam_toolbox),
                    transition_id=Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Start RViz2 alongside slam_toolbox.'),
        slam_toolbox,
        configure_event,
        activate_event,
        rviz,
    ])