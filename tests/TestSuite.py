#! /usr/bin/env python
# -*- coding: utf-8 -*-

import unittest
import JointsTests, ConstraintsTests, HomogeneousMatrixTests

tests = unittest.TestSuite([JointsTests.ts, ConstraintsTests.ts,
                            HomogeneousMatrixTests.ts])

unittest.TextTestRunner(verbosity=2).run(tests)
