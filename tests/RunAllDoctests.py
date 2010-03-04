# coding=utf-8
"""Running all doctest in /arboris as unittest."""

import unittest
import os
from os.path import splitext, basename, join as pjoin, walk, sep
import doctest
class DocTestCase:
    def __init__(self):
        '''
        Finds all the doctests files in arboris/, and run them as unittest.
        '''
        pymods = []
        for root, dirs, files in os.walk('arboris'):
            for file in files:
                package = '.'.join(root.split(sep))
                if file.endswith('.py') and file != '__init__.py':
                    pymods.append('.'.join([package, splitext(file)[0]]))

        suite = unittest.TestSuite()

        for mod in pymods:
            try:
                exec('import {0} as module'.format(mod))
                suite.addTest(doctest.DocTestSuite(module))
            except ImportError as ie:
                print "\n>>> ImportError:", ie
                print ">>> All doctest(s) related won't be executed.\n"
        unittest.TextTestRunner(verbosity=2).run(suite)

doctests = DocTestCase()
