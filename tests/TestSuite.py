#! /usr/bin/env python
# -*- coding: utf-8 -*-

import unittest
import JointsTests, ConstraintsTests

tests = unittest.TestSuite([JointsTests.ts, ConstraintsTests.ts])

unittest.TextTestRunner(verbosity=2).run(tests)
