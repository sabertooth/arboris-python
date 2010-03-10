#! /usr/bin/env python
# -*- coding: utf-8 -*-

import unittest
import JointsTests, ConstraintsTests, HomogeneousMatrixTests, Human36Tests
import UpdateDynamicTests, ControllersTests

tests = unittest.TestSuite([JointsTests.ts, ConstraintsTests.ts,
                            HomogeneousMatrixTests.ts, Human36Tests.ts,
                            UpdateDynamicTests.ts, ControllersTests.ts])

unittest.TextTestRunner(verbosity=2).run(tests)
