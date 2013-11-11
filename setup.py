#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
   packages=['uw_pr2_gripper_grasp_planner_cluster'],
   package_dir={'': 'src'}
)

setup(**d)
