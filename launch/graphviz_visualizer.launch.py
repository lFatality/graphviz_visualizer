# Copyright (c) 2021 Fynn Boyer
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    graph_config = os.path.join(
        get_package_share_directory('graphviz_visualizer'),
        'config',
        'graphs.yaml'
        )

    anomaly_flags_config = os.path.join(
            get_package_share_directory('graphviz_visualizer'),
            'config',
            'anomaly_flags.yaml'
            )
            
    node_names_config = os.path.join(
            get_package_share_directory('graphviz_visualizer'),
            'config',
            'node_names.yaml'
            )

    node = Node(
            package='graphviz_visualizer',
            namespace='',
            executable='graphviz_visualizer_executable',
            name='graphviz_visualizer_node',
            parameters=[graph_config, anomaly_flags_config, node_names_config],
            output='screen',
            emulate_tty=True
        )
        
    ld.add_action(node)
    return ld
