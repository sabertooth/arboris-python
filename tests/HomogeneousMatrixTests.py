# coding=utf-8

import unittest
from arboris.homogeneousmatrix import *
from numpy.linalg import norm
from ArborisTests import BaseTest
from numpy import arange, eye
from arboris.constraints import JointLimits, BallAndSocketConstraint 
from arboris.controllers import WeightController
from arboris.core import simplearm, simulate, Body, World
from arboris.joints import FreeJoint

class HomogeneousMatrixTestCase(BaseTest):
    
    def test(self):
        (az, ay, ax) = (3.14/6, 3.14/4, 3.14/3)
        self.assertTrue(norm(rotzyx(az,ay,ax) - 
                        dot(rotz(az),dot(roty(ay),rotx(ax)))) < 1e-10)

    def testTransl(self):
        self.assertListsAlmostEqual(transl(1., 2., 3.),
                                    [[ 1.,  0.,  0.,  1.],
                                     [ 0.,  1.,  0.,  2.],
                                     [ 0.,  0.,  1.,  3.],
                                     [ 0.,  0.,  0.,  1.]])

    def testRotzyx(self):
        self.assertListsAlmostEqual(rotzyx(3.14/6, 3.14/4, 3.14/3),
                   [[ 0.61271008,  0.27992274,  0.73907349,  0.        ],
                    [ 0.35353151,  0.73930695, -0.57309746,  0.        ],
                    [-0.70682518,  0.61242835,  0.35401931,  0.        ],
                    [ 0.        ,  0.        ,  0.        ,  1.        ]])

    def testRotzy(self):
        self.assertListsAlmostEqual(rotzy(3.14/6, 3.14/4),
                      [[ 0.61271008, -0.4997701 ,  0.61222235,  0.        ],
                       [ 0.35353151,  0.86615809,  0.35325009,  0.        ],
                       [-0.70682518,  0.        ,  0.70738827,  0.        ],
                       [ 0.        ,  0.        ,  0.        ,  1.        ]])

    def testRotzx(self):
        self.assertListsAlmostEqual(rotzx(3.14/6, 3.14/3),
                  [[ 0.86615809, -0.25011479,  0.43268088,  0.        ],
                   [ 0.4997701 ,  0.43347721, -0.74988489,  0.        ],
                   [ 0.        ,  0.86575984,  0.50045969,  0.        ],
                   [ 0.        ,  0.        ,  0.        ,  1.        ]])

    def testRotyx(self):
        self.assertListsAlmostEqual(rotyx(3.14/4, 3.14/3),
                      [[ 0.70738827,  0.61194086,  0.35373751,  0.        ],
                       [ 0.        ,  0.50045969, -0.86575984,  0.        ],
                       [-0.70682518,  0.61242835,  0.35401931,  0.        ],
                       [ 0.        ,  0.        ,  0.        ,  1.        ]])

    def testRotx(self):
        self.assertListsAlmostEqual(rotx(3.14/6),
                      [[ 1.        ,  0.        ,  0.        ,  0.        ],
                       [ 0.        ,  0.86615809, -0.4997701 ,  0.        ],
                       [ 0.        ,  0.4997701 ,  0.86615809,  0.        ],
                       [ 0.        ,  0.        ,  0.        ,  1.        ]])

    def testRoty(self):
        self.assertListsAlmostEqual(roty(3.14/6),
                  [[ 0.86615809,  0.        ,  0.4997701 ,  0.        ],
                   [ 0.        ,  1.        ,  0.        ,  0.        ],
                   [-0.4997701 ,  0.        ,  0.86615809,  0.        ],
                   [ 0.        ,  0.        ,  0.        ,  1.        ]])

    def testRotz(self):
        self.assertListsAlmostEqual(rotz(3.14/6),
                  [[ 0.86615809, -0.4997701 ,  0.        ,  0.        ],
                   [ 0.4997701 ,  0.86615809,  0.        ,  0.        ],
                   [ 0.        ,  0.        ,  1.        ,  0.        ],
                   [ 0.        ,  0.        ,  0.        ,  1.        ]])

    def testInv(self):
        H = array(
             [[ 0.70738827,  0.        , -0.70682518,  3.        ],
              [ 0.61194086,  0.50045969,  0.61242835,  4.        ],
              [ 0.35373751, -0.86575984,  0.35401931,  5.        ],
              [ 0.        ,  0.        ,  0.        ,  1.        ]])
        self.assertListsAlmostEqual(inv(H),
                  [[ 0.70738827,  0.61194086,  0.35373751, -6.3386158 ],
                   [ 0.        ,  0.50045969, -0.86575984,  2.32696044],
                   [-0.70682518,  0.61242835,  0.35401931, -2.09933441],
                   [ 0.        ,  0.        ,  0.        ,  1.        ]])

    def testAdjoint(self):
        H = array(
             [[ 0.70738827,  0.        , -0.70682518,  3.        ],
              [ 0.61194086,  0.50045969,  0.61242835,  4.        ],
              [ 0.35373751, -0.86575984,  0.35401931,  5.        ],
              [ 0.        ,  0.        ,  0.        ,  1.        ]])
        self.assertListsAlmostEqual(adjoint(H),
        [[ 0.70738827,  0., -0.70682518, 0. , 0. , 0. ],
         [ 0.61194086,  0.50045969,  0.61242835,  0. ,  0. , 0. ],
         [ 0.35373751, -0.86575984,  0.35401931,  0. ,  0. , 0. ],
         [-1.64475426, -5.96533781, -1.64606451, 0.70738827, 0., -0.70682518],
         [ 2.47572882,  2.59727952, -4.59618383,  0.61194086,  0.50045969,
                 0.61242835],
         [-0.9937305 ,  1.50137907,  4.66458577,  0.35373751, -0.86575984,
                 0.35401931]])

ts = unittest.TestSuite()
ts.addTest(HomogeneousMatrixTestCase('test'))
ts.addTest(HomogeneousMatrixTestCase('testTransl'))
ts.addTest(HomogeneousMatrixTestCase('testRotzyx'))
ts.addTest(HomogeneousMatrixTestCase('testRotzy'))
ts.addTest(HomogeneousMatrixTestCase('testRotzx'))
ts.addTest(HomogeneousMatrixTestCase('testRotyx'))
ts.addTest(HomogeneousMatrixTestCase('testRotx'))
ts.addTest(HomogeneousMatrixTestCase('testRoty'))
ts.addTest(HomogeneousMatrixTestCase('testRotz'))
ts.addTest(HomogeneousMatrixTestCase('testInv'))
ts.addTest(HomogeneousMatrixTestCase('testAdjoint'))