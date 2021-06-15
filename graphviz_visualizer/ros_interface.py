import os
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import UInt8MultiArray
from graphviz_visualizer_interfaces.msg import GraphInfo

import graphviz_visualizer.graphviz_visualizer as graphviz_visualizer

class GraphvizVisualizerNode(Node):
    def __init__(self):
        super().__init__("graphviz_visualizer_node")
        self.visualizer = None
        # params
        self.declare_parameters(
            namespace='',
            parameters=[
                ('graphs.names', None),
                ('graphs.initial_states', None),
                ('graphs.dotfile_template_paths', None),
                ('anomaly_flags', None),
                ('node_names', None),
            ]
        )
        # subscriptions
        self.init_subs()

    def init_subs(self):
        self.subscription = self.create_subscription(
            GraphInfo,
            'graph/states',
            self.graph_state_callback,
            10)

        self.subscription = self.create_subscription(
            UInt8MultiArray,
            'nodes_alive',
            self.node_liveliness_callback,
            1)

    def get_graph_config(self):
        """! Get the graph config defined in the .yaml file.
        
        @return list of dictionaries with the following fields:
            - name (str): The name of the graph
            - current_state (str): The current state of the graph
            - dotfile_template_path (str): The absolute path to the template dotfile of the graph
        """
        arr = []
        names = self.get_parameter('graphs.names').value
        initial_states = self.get_parameter('graphs.initial_states').value
        dotfile_template_paths = self.get_parameter('graphs.dotfile_template_paths').value

        assert len(names) == len(initial_states) == len(dotfile_template_paths),\
            "Incorrect parameters: Names (len %d), initial_states (len %d), dotfiles (len %d) must have the same number of entries "\
            % (len(names), len(initial_states), len(dotfile_template_paths))

        for i in range(len(names)):
            dic = {
                "name": names[i],
                "current_state": initial_states[i],
                "dotfile_template_path": os.path.join(get_package_share_directory('graphviz_visualizer'), dotfile_template_paths[i]),
            }
            arr.append(dic)

        return arr

    def get_node_names(self):
        """! Get the available node names from the .yaml file.

        @return array with node names where the array index indicates the node id
        """
        node_names = self.get_parameter('node_names').value
        return node_names

    def set_visualizer(self, visualizer):
        self.visualizer = visualizer

    def graph_state_callback(self, msg):
        """! Forward the new graph state to the visualizer to trigger the visual update 
        
        @param msg graph_visualizer/msg/GraphInfo topic message
        """
        self.visualizer.update_graph_state(msg.graph_name, msg.state_name)

    def node_liveliness_callback(self, msg):
        """! Topic callback to update the node liveliness display"""
        for i, _ in enumerate(self.get_node_names()):
            if i in msg.data:
                self.visualizer.update_nodes_liveliness(i, 1)
            else:
                self.visualizer.update_nodes_liveliness(i, 0)
