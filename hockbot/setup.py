#!/usr/bin/env python

# Use catkin, don't run manually

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['hockbot'],
    package_dir={'': 'src'}
)

setup(**setup_args)
