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

from jinja2 import Template
from pathlib import Path

import io
import os
import pydot
import sys


def create_graph_svg(graph_dict):
    """! Generate an .svg file in RAM and return it.
    
    @param graph_dict Dictionary with graph information
    """
    path = get_output_path_from_template_path(graph_dict['dotfile_template_path'])

    with io.open(path, 'rt', encoding='utf-8') as stdin:
        stdout, stderr_data, process = pydot.call_graphviz(
            program='dot',
            arguments=['-Tsvg', ],
            working_dir=os.path.dirname(path),
            stdin=stdin,
        )
    
    assert process.returncode == 0, stderr_data

    return stdout

def generate_dotfile(graph_dict, active_state : str):
    """! Render the jinja2 template into a .dot file and write it on disk

    @param graph_dict Dictionary with graph information
    @param active_state The active state within the graph
    """
    input_path = graph_dict['dotfile_template_path']
    output_path = get_output_path_from_template_path(graph_dict['dotfile_template_path'])
    # print('Load node info template "%s".' % (input_path))

    with open('%s' % (input_path)) as f:
        tmpl = Template(f.read())

        # apply changes to template
        out = tmpl.render(
            active_state = active_state
        )

        # create and write new file based on the template
        f = open('%s' % (output_path), "w")
        f.write(out)
        f.close()

def get_output_path_from_template_path(template_path):
    """! Create the output path for the .dot file from the template path

    Example:
    template path: "/path/to/template/myTemplate.j2"
    output path: "/path/to/template/generated/myTemplate.dot"

    @param template_path The path to the dotfile template
    @return The path were to generate the .dot file
    """
    file_name, _ = os.path.splitext(os.path.basename(template_path))
    return os.path.join(os.path.dirname(template_path), 'generated', file_name + '.dot')
