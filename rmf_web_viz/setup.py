from setuptools import setup
import os
from glob import glob

package_name = 'rmf_web_viz'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.xml')),
        (os.path.join('lib', package_name, 'templates'), glob(package_name + '/templates/*')),
        (os.path.join('lib', package_name, 'static'), glob(package_name + '/static/*')),
    ],
    install_requires=['setuptools', 'Flask', 'python-socketio', 'eventlet'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='A simple web visualizer for Open-RMF',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'web_viz = rmf_web_viz.web_viz:main',
        ],
    },
)

