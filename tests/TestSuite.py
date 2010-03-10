#! /usr/bin/env python
# -*- coding: utf-8 -*-

import unittest
import JointsTests, ConstraintsTests, HomogeneousMatrixTests, Human36Tests
import UpdateDynamicTests, ControllersTests, FrameTests

tests = unittest.TestSuite([JointsTests.ts, ConstraintsTests.ts,
                            HomogeneousMatrixTests.ts, Human36Tests.ts,
                            UpdateDynamicTests.ts, ControllersTests.ts,
                            FrameTests.ts])

unittest.TextTestRunner(verbosity=2).run(tests)
