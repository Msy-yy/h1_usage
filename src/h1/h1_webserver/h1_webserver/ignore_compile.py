#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Software License Agreement (BSD)
#
# @author    Salman Omar Sohail <support@mybotshop.de>
# @copyright (c) 2025, MYBOTSHOP GmbH, Inc., All rights reserved.

from setuptools import setup, Extension
from setuptools.command.build_ext import build_ext
from Cython.Build import cythonize  # Added import for cythonize

python_file_names = ['libaudiogen', 'librosnode', 'libwebserver']

ext_modules = [
    Extension(
        name,
        [name + ".py"],
    ) for name in python_file_names
]

setup(
    name=python_file_names[0] + '_lib',  
    cmdclass={'build_ext': build_ext},
    ext_modules=cythonize(
        ext_modules,
        compiler_directives={'language_level': '3str'},  
        annotate=False                                    
    )
)