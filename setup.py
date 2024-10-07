#!/usr/bin/env python3

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# for your packages to be recognized by python
d = generate_distutils_setup(
 packages=['rule_based_gpsr_ros'],
 package_dir={'rule_based_gpsr_ros': 'ros/src/rule_based_gpsr_ros'}
)

setup(**d)
