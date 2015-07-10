#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages = ['vigir_behavior_flexbe_inner_test_behavior'],
    package_dir = {'': 'src'}
)

setup(**d)