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
    
    rviz_config_file = PathJoinSubstitution([
        pkg_autonav_rviz,
        'rviz',
        'gz_autonav_mapping.rviz'
    ])

    gz_deliverer_config_file = PathJoinSubstitution([
        pkg_autonav_navigation,
        'config',
        'gz_autonav_mapping.yaml'
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

    slam_toolbox_mapping = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        remappings=[('/tf', '/autonav/tf'),
            ('/tf_static', '/autonav/tf_static'),
            ('/map', '/autonav/map'),
            ('/map_metadata', '/autonav/map_metadata')],
        name='slam_toolbox',
        parameters=[gz_deliverer_config_file],
        namespace='autonav',
        output='screen',
    )

    rviz = GroupAction(
        actions=[
            SetRemap(src='/tf', dst='/autonav/tf'),
            SetRemap(src='/tf_static', dst='/autonav/tf_static'),

            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz',
                output='screen',
                arguments=['-d', rviz_config_file],
            )
        ]
    )

    return LaunchDescription([
        pcl_to_scan,
        slam_toolbox_mapping,
        rviz
    ])