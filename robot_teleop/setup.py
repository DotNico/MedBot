#!/usr/bin/python3

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
     packages=['robot_teleop'],
     package_dir={'': 'src'},
     install_requires=[]
)

setup(**setup_args)