from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing
 
import os
import pytest
import unittest
from time import sleep

# this test is used to just launch the GraphvizVisualizer and see if that works without errors.
# the test will indicate if problems exist (smoke).

@pytest.mark.ros_test
def generate_test_description():
 
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

    graphviz_visualizer_node = Node(
            package='graphviz_visualizer',
            namespace='',
            executable='graphviz_visualizer_executable',
            name='graphviz_visualizer_node',
            parameters=[graph_config, anomaly_flags_config, node_names_config],
            output='screen',
            emulate_tty=True
        )

    context = {'graphviz_visualizer_node': graphviz_visualizer_node}
 
    launch_description = LaunchDescription([
        graphviz_visualizer_node,
        # Start tests right away - no need to wait for anything
        launch_testing.actions.ReadyToTest()])
 
    return launch_description, context

class TestTaskManagement(unittest.TestCase):

    def test_sleep(self, proc_output, proc_info, graphviz_visualizer_node):
        # no real tests are happening.
        # but we need a delay because the shutdown signal comes too fast otherwise
        sleep(5)

@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):
 
    def test_exit_code(self, proc_output, proc_info, graphviz_visualizer_node):
        # Check that process exits cleanly with exit code 0
        launch_testing.asserts.assertExitCodes(proc_info, process=graphviz_visualizer_node)