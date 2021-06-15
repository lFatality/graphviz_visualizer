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
import os
from threading import Thread

from PySide2 import QtCore
from PySide2.QtWidgets import QApplication
from PySide2.QtCore import Qt, QCoreApplication

script_dir = os.path.dirname( __file__ )
graphviz_visualizer_dir = os.path.join( script_dir, '..' )
sys.path.append( graphviz_visualizer_dir )

import graphviz_visualizer.graphviz_visualizer as graphviz_visualizer

def get_path_to_executed_script():
    """! Get the path to the executed script """
    return os.path.dirname( __file__ )

def get_dotfile_template_path(template_file_name):
    """! Get the path to a dotfile template file

    @param template_file_name The name of the template file
    """
    return os.path.join(get_path_to_executed_script(), "..", "assets", "templates", template_file_name)

def get_ui_file_path(ui_file_name):
    """! Get the path to a dotfile template file

    @param template_file_name The name of the ui file
    """
    return os.path.join(get_path_to_executed_script(), "..", "assets", "ui", ui_file_name)

def prompt_user_input(graph_vis):
    """! Ask the user for input to update the graph """
    while True:
        print("Provide a new state for a graph.")
        graph_name = input("Enter graph name:")
        state_name = input("Enter state:")

        graph_vis.update_graph_state(graph_name, state_name)

        print("")

def main(args=None):

    graph_config = [
        {"name": "high_level_fsm",
         "current_state": "Init",
         "dotfile_template_path": get_dotfile_template_path("task_management_fsm.j2")
        },
        {"name": "locomotion",
         "current_state": "NoDataYet",
         "dotfile_template_path": get_dotfile_template_path("task_management_fsm.j2")
        },
        {"name": "example_graph",
         "current_state": "State0",
         "dotfile_template_path": get_dotfile_template_path("example_graph.j2")
        },
        {"name": "example_tree",
         "current_state": "chase_ghost",
         "dotfile_template_path": get_dotfile_template_path("example_tree.j2")
        }
    ]

    node_names = ["Node1", "Node2","Node3","Node4","Node5","Node6","Node7","Node8","Node9","Node10"]

    QtCore.QCoreApplication.setAttribute(QtCore.Qt.AA_ShareOpenGLContexts)

    app = QApplication([])
    setup_interrupt_handling()

    ui_path = get_ui_file_path("graphviz_visualizer.ui")

    graph_vis = graphviz_visualizer.GraphvizVisualizer(graph_config, node_names, ui_path)

    graph_vis.ui.show()

    t = Thread(target = prompt_user_input, args = (graph_vis, ), daemon=True)
    t.start()

    app.exec_()

    # t.join()
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
