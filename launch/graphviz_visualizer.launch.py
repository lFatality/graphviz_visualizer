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
