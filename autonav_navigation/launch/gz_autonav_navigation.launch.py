# Copyright (c) 2025, Dmitry Anikin <dmitry.anikin@proton.me>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its contributors
#    may be used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from launch import LaunchDescription
from launch.actions import GroupAction
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, SetRemap

def generate_launch_description():
    pkg_autonav_navigation = FindPackageShare('autonav_navigation')
    pkg_autonav_rviz = FindPackageShare('autonav_rviz')
    pkg_nav2_bt_navigator = FindPackageShare('nav2_bt_navigator')

    rviz_config_file = PathJoinSubstitution([
        pkg_autonav_rviz,
        'rviz',
        'gz_autonav_navigation.rviz'
    ])

    gz_deliverer_config_file = PathJoinSubstitution([
        pkg_autonav_navigation,
        'config',
        'gz_autonav_navigation.yaml'
    ])

    gz_deliverer_map_file = PathJoinSubstitution([
        pkg_autonav_navigation,
        'data',
        'gz_autonav.yaml'
    ])

    gz_deliverer_nav2_to_pose_bt_xml = PathJoinSubstitution([
        pkg_nav2_bt_navigator,
        'behavior_trees',
        'navigate_to_pose_w_replanning_and_recovery.xml'
    ])

    gz_deliverer_nav2_through_poses_bt_xml = PathJoinSubstitution([
        pkg_nav2_bt_navigator,
        'behavior_trees',
        'navigate_through_poses_w_replanning_and_recovery.xml'
    ])

    pcl_to_scan = Node(
        package='pointcloud_to_laserscan', 
        executable='pointcloud_to_laserscan_node',
        remappings=[('cloud_in', '/autonav/sensors/lidar3d_0/points'),
                    ('scan', '/autonav/navigation/scan'),
                    ('/tf', '/autonav/tf'),
                    ('/tf_static', '/autonav/tf_static'),],
        parameters=[gz_deliverer_config_file],
        namespace="autonav",
        name='pointcloud_to_laserscan',
        output='screen'
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        namespace='autonav',
        parameters=[gz_deliverer_config_file]
    )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        namespace='autonav',
        output='screen',
        parameters=[
            gz_deliverer_config_file,
            {'yaml_filename': gz_deliverer_map_file}
        ],  
        remappings=[
            ('/tf', '/autonav/tf'),
            ('/tf_static', '/autonav/tf_static'),
        ]
    )

    costmap_filter_info_server = Node(
        package='nav2_map_server',
        executable='costmap_filter_info_server',
        name='costmap_filter_info_server',
        namespace='autonav',
        output='screen',
        parameters=[gz_deliverer_config_file]
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        namespace='autonav',
        output='screen',
        parameters=[gz_deliverer_config_file],
        remappings=[
             ('/tf', '/autonav/tf'),
             ('/tf_static', '/autonav/tf_static'),
        ]
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        namespace='autonav',
        output='screen',
        parameters=[gz_deliverer_config_file],
        remappings=[
            ('/tf', '/autonav/tf'),
            ('/tf_static', '/autonav/tf_static'),
        ]
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        namespace='autonav',
        output='screen',
        parameters=[
            gz_deliverer_config_file,
            {
                'default_nav_to_pose_bt_xml': gz_deliverer_nav2_to_pose_bt_xml,
                'default_nav_through_poses_bt_xml' : gz_deliverer_nav2_through_poses_bt_xml,
            }
        ],
        remappings=[
            ('/tf', '/autonav/tf'),
            ('/tf_static', '/autonav/tf_static'),
        ]
    )

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        namespace='autonav',
        output='screen',
        parameters=[gz_deliverer_config_file],
        remappings=[
            ('/tf', '/autonav/tf'),
            ('/tf_static', '/autonav/tf_static')
        ]
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        namespace='autonav',
        output='screen',
        parameters=[gz_deliverer_config_file],
        remappings=[
            ('/tf', '/autonav/tf'),
            ('/tf_static', '/autonav/tf_static')
        ]
    )

    rviz = GroupAction(
        actions=[
            SetRemap(src='/tf', dst='/autonav/tf'),
            SetRemap(src='/tf_static', dst='/autonav/tf_static'),
            SetRemap(src='/clicked_point', dst='/autonav/clicked_point'),
            SetRemap(src='/goal_pose', dst='/autonav/goal_pose'),
            SetRemap(src='/initialpose', dst='/autonav/initialpose'),

            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz',
                namespace='autonav',
                output='screen',
                arguments=['-d', rviz_config_file],
            )
        ]
    )

    return LaunchDescription([
        pcl_to_scan,
        lifecycle_manager,
        map_server,
        costmap_filter_info_server,
        amcl,
        planner_server,
        controller_server,
        bt_navigator,
        behavior_server,
        rviz
    ])
