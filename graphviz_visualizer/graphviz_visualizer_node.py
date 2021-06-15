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

# This Python file uses the following encoding: utf-8

import sys
import signal
import rclpy
import os
from threading import Thread
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import String

from PySide2 import QtCore
from PySide2.QtWidgets import QApplication
from PySide2.QtCore import Qt, QCoreApplication

import graphviz_visualizer.graphviz_visualizer as graphviz_visualizer
import graphviz_visualizer.ros_interface as ros_interface

def main(args=None):
    rclpy.init(args=args)

    node = ros_interface.GraphvizVisualizerNode()
    graph_config = node.get_graph_config()
    node_names = node.get_node_names()

    QtCore.QCoreApplication.setAttribute(QtCore.Qt.AA_ShareOpenGLContexts)

    app = QApplication([])
    setup_interrupt_handling()

    ui_path = os.path.join(get_package_share_directory('graphviz_visualizer'), "assets/ui/graphviz_visualizer.ui")

    graph_vis = graphviz_visualizer.GraphvizVisualizer(graph_config, node_names, ui_path)
    node.set_visualizer(graph_vis)

    graph_vis.ui.show()

    t = Thread(target = rclpy.spin, args = (node, ), daemon=True)
    t.start()

    app.exec_()

    node.destroy_node()
    rclpy.shutdown()
    t.join()
    sys.exit()

# Interrupt handling code from here: https://coldfix.eu/2016/11/08/pyqt-boilerplate/
# Call this function in your main after creating the QApplication
def setup_interrupt_handling():
    """Setup handling of KeyboardInterrupt (Ctrl-C) for PyQt."""
    signal.signal(signal.SIGINT, _interrupt_handler)
    # Regularly run some (any) python code, so the signal handler gets a
    # chance to be executed:
    safe_timer(50, lambda: None)


# Define this as a global function to make sure it is not garbage
# collected when going out of scope:
def _interrupt_handler(signum, frame):
    """Handle KeyboardInterrupt: quit application."""
    QCoreApplication.quit()


def safe_timer(timeout, func, *args, **kwargs):
    """
    Create a timer that is safe against garbage collection and overlapping
    calls. See: http://ralsina.me/weblog/posts/BB974.html
    """
    def timer_event():
        try:
            func(*args, **kwargs)
        finally:
            QtCore.QTimer.singleShot(timeout, timer_event)
    QtCore.QTimer.singleShot(timeout, timer_event)

if __name__ == "__main__":
    main()
