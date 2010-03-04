# coding: utf-8
__author__ = ("Sébastien BARTHÉLEMY <sebastien.barthelemy@gmail.com>")

from distutils.core import setup, Command
from glob import glob
import os
from os.path import splitext, basename, join as pjoin, walk, sep
import doctest
import subprocess

class TestCommand(Command):
    user_options = []

    def initialize_options(self):
        pass

    def finalize_options(self):
        pass

    def run(self):
        ''''
        Run all the tests.

        Finds all the doctests files in both arboris/ and tests/ 
        and run them as a unittest test suite.
        '''

        for rst in glob(pjoin('tests', '*.rst')):
            doctest.testfile(rst)
        
        p = subprocess.Popen(args=["python","tests/RunAllDoctests.py"])
        p.wait()

cmdclass = {'test': TestCommand}

try:
    from sphinx.setup_command import BuildDoc
    cmdclass['build_doc'] = BuildDoc
except ImportError:
    pass

version = '0.1.0pre4'

setup(
    name='arboris',
    author='Sébastien BARTHÉLEMY',
    version=version,
    packages=['arboris',
              'arboris.robots'],
    cmdclass=cmdclass,
    package_data = {'arboris':['doc']}
)

