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

from PySide2 import QtCore
from PySide2.QtWidgets import QApplication, QFormLayout, QWidget, QVBoxLayout, QLineEdit
from PySide2.QtCore import QFile, QByteArray
from PySide2.QtSvg import QSvgWidget
from PySide2.QtUiTools import QUiLoader
from PySide2.QtGui import QPainter, QBrush, QPen, QColor, QPalette
from PySide2.QtCore import Qt

import sys
from PySide2.QtGui import QPixmap
from PySide2.QtWidgets import QMainWindow, QLabel

import graphviz_visualizer.graphviz_tools as graphviz_tools
from graphviz_visualizer.circle_drawer import CircleDrawer

class GraphvizVisualizer(object):
    def __init__(self, graph_dict_list, node_names, ui_path, parent=None, parent_widget=None):
        """ Constructs the graphviz visualizer

        This GUI provides information about:
        - the active state of the graphs
        - the liveliness of the nodes of the system

        @param graph_dict_list List of dictionaries, each specifying a graph
            Each dictionary has 4 entries:
            - name (str): The name of the graph
            - current_state (str): The current state of the graph
            - dotfile_template_path (str): The absolute path to the template dotfile of the graph
        @param node_names Array containing the node names where the index within the array indicates
            the node id
        @param ui_path Path to the .ui file, defining the structure of the GUI
        @param parent
        @param parent_widget
        """
        super().__init__()
        self.graph_dict_list = graph_dict_list
        self.ui = self.load_ui(ui_path, parent_widget)
        self.title = "GraphvizVisualizer"
        self.ui.setWindowTitle(self.title)

        self.graphStateTextOverviewTexts = {} # the lineEdits containing the graph state in the text overview
        self.livelinessCircleDrawers = {} # the LEDs indicating the liveliness of nodes in the text overview
        self.visualOverviewSvgWidgets = {} # svg widgets in the visual overview tab containing graph visualizations
        self.singleTabViewSvgWidgets = {} # svg widgets in the individual graph tab containing graph visualizations

        for i in range(len(graph_dict_list)):
            graph_dict = graph_dict_list[i]
            
            self.add_graph_state_text_overview(graph_dict)
            self.add_visual_overview_widget(graph_dict, i)
            self.add_tab_svg_widget(graph_dict)

            # display initial state of the graph
            self.update_graph_state(graph_dict['name'], graph_dict['current_state'], True)

        self.add_liveliness_overview(node_names)

    def load_ui(self, ui_path, parent_widget):
        loader = QUiLoader()
        ui_file = QFile(ui_path)
        ui_file.open(QFile.ReadOnly)
        ui = loader.load(ui_file, parent_widget)
        ui_file.close()
        return ui

    def add_visual_overview_widget(self, graph_dict, graph_num):
        # add graph as a widget (text + svg)
        # create svg widget
        svg_widget = QSvgWidget()
        self.visualOverviewSvgWidgets[graph_dict['name']] = svg_widget
        svg_widget.setMinimumHeight(int(300))
        # create text
        layout = QVBoxLayout()
        label = QLabel(self.ui)
        label.setText(graph_dict['name'])
        label.setMaximumHeight(25)
        label.setAlignment(QtCore.Qt.AlignCenter | QtCore.Qt.AlignVCenter)
        # add svg + text to vertical layout
        layout.addWidget(label)
        layout.addWidget(svg_widget)
        # add layout to grid layout containing all graphs
        self.ui.visualOverviewGraphGridLayout.addLayout(layout, graph_num / 2 + 1, graph_num % 2)

    def add_tab_svg_widget(self, graph_dict):
        """! Add a tab to the GraphvizVisualizer to visually display the state of a single graph

        @param graph_dict Dictionary containing information about the graph
        """
        svg_widget = QSvgWidget()
        self.singleTabViewSvgWidgets[graph_dict['name']] = svg_widget
        self.ui.tabWidget.addTab(svg_widget, graph_dict['name'])

    def add_liveliness_overview(self, node_names):
        """! Add the LEDs and text showing the alive status of nodes

        @param node_names Array containing the names of available nodes
        """
        for i, node_name in enumerate(node_names):
            led = CircleDrawer(25, Qt.red)
            node_name_label = QLabel(self.ui)
            node_name_label.setText(node_name)
            node_name_label.setAlignment(QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter)

            liveliness_layout = QFormLayout()
            liveliness_layout.addRow(led, node_name_label)
            # split nodes into 2 columns
            if (i < len(node_names)/2):
                self.ui.livelinessTextOverviewGridLayout.addLayout(liveliness_layout, i, 0)
            else:
                self.ui.livelinessTextOverviewGridLayout.addLayout(liveliness_layout, i - len(node_names)/2, 1)
            self.livelinessCircleDrawers[i] = led

    def add_graph_state_text_overview (self, graph_dict):
        """! Add the graph state text field of a single graph in the text overview tab

        @param The dictionary of the graph to add
        """
        lineEdit = QLineEdit(self.ui)
        lineEdit.setReadOnly(True)

        label = QLabel(self.ui)
        label.setText(graph_dict['name'])
        label.setAlignment(QtCore.Qt.AlignCenter | QtCore.Qt.AlignVCenter)
        self.ui.graphStateTextOverviewFormLayout.addRow(label, lineEdit)

        self.graphStateTextOverviewTexts[graph_dict['name']] = lineEdit

    def update_graph_state(self, graph_name, state_name, force=False):
        """! Update the state of a graph in the text & visual displays

        @param graph_name The name of the graph to update
        @param state_name The new state of the Graph. If this state is equal to the current
            state of the Graph, no action will be taken to avoid spending unnecessary resources
        @param force Update the state no matter the current state. This is mainly used to
            set up the displays with the initial state.
        """
        graph_dict = self.get_graph_dict_from_name(graph_name)

        if not graph_dict:
            # unknown graph name
            print("unknown graph name %s" % graph_name)
            return

        if graph_dict['current_state'] == state_name and not force:
            # dont update if state is not new
            # allow forcing update to create initial state
            return

        graph_dict['current_state'] = state_name

        self.graphStateTextOverviewTexts[graph_dict['name']].setText(state_name)

        graphviz_tools.generate_dotfile(graph_dict, state_name)
        svg = graphviz_tools.create_graph_svg(graph_dict)
        self.visualOverviewSvgWidgets[graph_dict['name']].load(svg)
        self.singleTabViewSvgWidgets[graph_dict['name']].load(svg)
        # self.svgWidgets[graph_dict['name']].resize(100, 100)

    def update_nodes_liveliness(self, node_id, status):
        """! Update the visual displays of the node liveliness

        @param node_id The id of the node
        @param status 0 if dead, 1 if alive, 2 if unknown
        """
        if node_id in self.livelinessCircleDrawers.keys():
            if (0 == status):
                color = Qt.red
            elif (1 == status):
                color = Qt.green
            elif (2 == status):
                color = Qt.yellow
            else:
                print("Invalid node liveliness status")
                return
            self.livelinessCircleDrawers[node_id].color_ = color
            self.livelinessCircleDrawers[node_id].update()
        else:
            print("Invalid node id to update liveliness (id: %d)" % (node_id))

    def get_graph_dict_from_name(self, graph_name):
        """! Retrieve the graph dictionary belonging to the graph name

        @param graph_name The name of the Graph
        @return Dictionary containing information about the Graph
        """
        for graph_dict in self.graph_dict_list:
            if graph_dict['name'] == graph_name:
                return graph_dict

        return None
