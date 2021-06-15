import os
from glob import glob
from setuptools import setup

package_name = 'graphviz_visualizer'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'assets/ui'), glob('assets/ui/*.ui')),
        (os.path.join('share', package_name, 'assets/templates'), glob('assets/templates/*.j2')),
        (os.path.join('share', package_name, 'assets/templates/generated'), ['assets/templates/generated/empty']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fynn',
    maintainer_email='fynn-boyer@web.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'graphviz_visualizer_executable = graphviz_visualizer.graphviz_visualizer_node:main'
        ],
    },
)
