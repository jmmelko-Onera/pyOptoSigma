#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import sys
if sys.version_info[0] < 3:
    raise Exception('This module only supports Python 3.4 or later. Try use `python3 setup.py`')

from setuptools import setup

long_description = '''
Python module for operating stepping moter stages manufactured by OptoSigma and Sigma Koki.

Requirements
------------
* Python 3.4 or later. (Python 2.x are not supported.)
* pyserial

Setup
-----
::

    $ sudo pip3 pyserial pyOptoSigma
    
* In Ubuntu systems, add an user to dialout group.

::

    $ sudo gpasswd -a [username] dialout
    
History
-------
0.2 (2026-01-11)
~~~~~~~~~~~~~~~
* Branching by jmmelkon
* Adaptation to Opto-Sigma and MAC04 controller
-------
0.1 (2016-5-28)
~~~~~~~~~~~~~~~
* First release.
* Supports SHOT-702, SHOT-302GS, SHOT-304GS controllers.
* Supports SGSP, OSMS series stages.
'''

classifiers = [
    'Development Status :: 2 - Pre-Alpha',
    'Intended Audience :: Manufacturing',
    'Intended Audience :: Science/Research',
    'License :: OSI Approved :: MIT License',
    'Programming Language :: Python :: 3 :: Only'
]

setup(
    name = 'pyOptoSigma',
    version = '0.2',
    py_modules = ['pyOptoSigma'],
    description = 'Python module for operating stepping moter stages manufactured by OptoSigma and Sigma Koki.',
    long_description = long_description,
    classifiers = classifiers,
    keywords=['OptoSigma', 'Sigma Koki', 'シグマ光機', 'translation stages', 'rotation stages'],
    author = 'Jean-Michel Melkonian',
    author_email = 'jmmelko@gmail.com',
    url = 'TBD',
    license='MIT'
    )