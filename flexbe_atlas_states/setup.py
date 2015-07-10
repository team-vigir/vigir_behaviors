#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages = ['flexbe_atlas_states', 'flexbe_atlas_states.proxy'],
    package_dir = {'': 'src'}
)

setup(**d)
