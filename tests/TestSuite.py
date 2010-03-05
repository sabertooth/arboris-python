#! /usr/bin/env python
# -*- coding: utf-8 -*-

import unittest
import JointsTests

tests = unittest.TestSuite([JointsTests.ts])

unittest.TextTestRunner(verbosity=2).run(tests)
