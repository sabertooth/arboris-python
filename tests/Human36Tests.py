# -*- coding: utf-8 -*-

import unittest
from arboris.robots.human36 import _humans_anatomical_lengths
from numpy import zeros
from numpy.linalg import norm
from ArborisTests import BaseTest
from numpy import arange, eye
from arboris.constraints import JointLimits, BallAndSocketConstraint 
from arboris.controllers import WeightController
from arboris.core import simplearm, simulate, Body, World
from arboris.joints import FreeJoint
from arboris.robots.human36 import _humans_tags
from numpy import zeros
from arboris.core import World
from arboris.robots.human36 import _human36
from numpy import zeros
from numpy import dot
from arboris.robots.human36 import add_human36
from arboris.core import World

class Human36TestCase(BaseTest):
    
    def anatomicalLengthTest(self):
        def convert_to_array(lengths):
            L = zeros((len(lengths)))
            for length in lengths:
                L[length['HumansId']-1] = length["Value"]
            return L

        self.assertListsAlmostEqual(convert_to_array(_humans_anatomical_lengths(1.741)),
                      [ 0.3612575,  0.0915766,  0.1744482,  0.0386502,  0.4340313,
                        0.4221925,  0.170618 ,  0.0127   ,  0.1831532,  0.0181064,
                        0.2127595,  0.0915766,  0.2816938,  0.0386502,  0.4340313,
                        0.4221925,  0.170618 ,  0.0127   ,  0.1831532,  0.0181064,
                        0.2127595,  0.0915766,  0.2816938,  0.2688104,  0.2688104,
                        0.241999 ,  0.1899431,  0.1899431,  0.2428695,  0.2580162,
                        0.2580162])

        self.assertListsAlmostEqual(convert_to_array(_humans_anatomical_lengths(0.)),
                        [ 0.    ,  0.    ,  0.    ,  0.    ,  0.    ,  0.    ,  0.    ,
                          0.0127,  0.    ,  0.    , -0.0127,  0.    ,  0.    ,  0.    ,
                          0.    ,  0.    ,  0.    ,  0.0127,  0.    ,  0.    , -0.0127,
                          0.    ,  0.    ,  0.    ,  0.    ,  0.    ,  0.    ,  0.    ,
                          0.    ,  0.    ,  0.    ])
                    
                    
    def tagsTest(self):
        def convert_to_array(tags):
            t = zeros((len(tags),3))
            for tag in tags:
                t[tag['HumansId']-1,:] = tag["Position"][:]
            return t
        self.assertListsAlmostEqual(convert_to_array(_humans_tags(1.741)),
        [[ 0.2150135, -0.0386502,  0.       ],
               [-0.0431768, -0.0386502,  0.       ],
               [ 0.1152542, -0.0386502,  0.0531005],
               [ 0.1152542, -0.0386502, -0.0531005],
               [ 0.       , -0.4340313,  0.0433509],
               [ 0.       , -0.4221925,  0.050489 ],
               [ 0.       ,  0.       ,  0.076604 ],
               [ 0.0471811,  0.0637206,  0.1213477],
               [ 0.2150135, -0.0386502,  0.       ],
               [-0.0431768, -0.0386502,  0.       ],
               [ 0.1152542, -0.0386502, -0.0531005],
               [ 0.1152542, -0.0386502,  0.0531005],
               [ 0.       , -0.4340313, -0.0433509],
               [ 0.       , -0.4221925, -0.050489 ],
               [ 0.       ,  0.       , -0.076604 ],
               [ 0.0471811,  0.0637206, -0.1213477],
               [ 0.2122279,  0.       ,  0.       ],
               [ 0.1831532,  0.170618 ,  0.       ],
               [-0.0915766,  0.0525782,  0.2127595],
               [ 0.       , -0.2816938,  0.0367351],
               [ 0.       , -0.2668953,  0.0576271],
               [ 0.       , -0.1899431,  0.       ],
               [-0.0915766,  0.0525782, -0.2127595],
               [ 0.       , -0.2816938, -0.0367351],
               [ 0.       , -0.2668953, -0.0576271],
               [ 0.       , -0.1899431,  0.       ],
               [ 0.0915766,  0.241999 ,  0.       ],
               [ 0.       ,  0.2428695,  0.       ]])

        self.assertListsAlmostEqual(convert_to_array(_humans_tags(0.)),
        [[ 0.    , -0.    ,  0.    ],
               [-0.    , -0.    ,  0.    ],
               [ 0.    , -0.    ,  0.    ],
               [ 0.    , -0.    , -0.    ],
               [ 0.    , -0.    ,  0.    ],
               [ 0.    , -0.    ,  0.    ],
               [ 0.    ,  0.    ,  0.    ],
               [ 0.    ,  0.    ,  0.    ],
               [ 0.    , -0.    ,  0.    ],
               [-0.    , -0.    ,  0.    ],
               [ 0.    , -0.    , -0.    ],
               [ 0.    , -0.    ,  0.    ],
               [ 0.    , -0.    , -0.    ],
               [ 0.    , -0.    , -0.    ],
               [ 0.    ,  0.    ,  0.    ],
               [ 0.    ,  0.    , -0.    ],
               [ 0.    ,  0.    ,  0.    ],
               [ 0.    ,  0.    ,  0.    ],
               [-0.    ,  0.    , -0.0127],
               [ 0.    , -0.    ,  0.    ],
               [ 0.    , -0.    ,  0.    ],
               [ 0.    , -0.    ,  0.    ],
               [-0.    ,  0.    ,  0.0127],
               [ 0.    , -0.    , -0.    ],
               [ 0.    , -0.    , -0.    ],
               [ 0.    , -0.    ,  0.    ],
               [ 0.    ,  0.    ,  0.    ],
               [ 0.    ,  0.    ,  0.    ]])

    def test(self):
        w = World()
        (bodies, tags) = _human36(w)
        w.update_geometric()
        def tag_positions(tag_frames):
            pos_dict= {}
            for (key, val) in tag_frames.iteritems():
                pos_dict[key] = dot(val.body.pose, val._bpose)[0:3,3]

            pos_array = zeros((len(tag_frames),3))
            for t in _humans_tags(1.741):
                pos_array[t['HumansId']-1,:] = pos_dict[t['HumansName']]
            return pos_array
        p = tag_positions(tags)
        
        self.assertListsAlmostEqual(p,
        [[  2.15013500e-01,   2.08166817e-17,   8.72241000e-02],
               [ -4.31768000e-02,   2.08166817e-17,   8.72241000e-02],
               [  1.15254200e-01,   2.08166817e-17,   1.40324600e-01],
               [  1.15254200e-01,   2.08166817e-17,   3.41236000e-02],
               [  0.00000000e+00,   3.86502000e-02,   1.30575000e-01],
               [  0.00000000e+00,   4.72681500e-01,   1.37713100e-01],
               [  0.00000000e+00,   8.94874000e-01,   1.63828100e-01],
               [  4.71811000e-02,   9.58594600e-01,   1.21347700e-01],
               [  2.15013500e-01,   2.08166817e-17,  -8.72241000e-02],
               [ -4.31768000e-02,   2.08166817e-17,  -8.72241000e-02],
               [  1.15254200e-01,   2.08166817e-17,  -1.40324600e-01],
               [  1.15254200e-01,   2.08166817e-17,  -3.41236000e-02],
               [  0.00000000e+00,   3.86502000e-02,  -1.30575000e-01],
               [  0.00000000e+00,   4.72681500e-01,  -1.37713100e-01],
               [  0.00000000e+00,   8.94874000e-01,  -1.63828100e-01],
               [  4.71811000e-02,   9.58594600e-01,  -1.21347700e-01],
               [  1.20651300e-01,   1.25613150e+00,   0.00000000e+00],
               [  9.15766000e-02,   1.42674950e+00,   0.00000000e+00],
               [  0.00000000e+00,   1.47932770e+00,   2.25459500e-01],
               [  0.00000000e+00,   1.16316210e+00,   2.62194600e-01],
               [  0.00000000e+00,   8.96266800e-01,   2.83086600e-01],
               [  0.00000000e+00,   7.04408600e-01,   2.25459500e-01],
               [  0.00000000e+00,   1.47932770e+00,  -2.25459500e-01],
               [  0.00000000e+00,   1.16316210e+00,  -2.62194600e-01],
               [  0.00000000e+00,   8.96266800e-01,  -2.83086600e-01],
               [  0.00000000e+00,   7.04408600e-01,  -2.25459500e-01],
               [  0.00000000e+00,   1.49813050e+00,   0.00000000e+00],
               [  0.00000000e+00,   1.74100000e+00,   0.00000000e+00]])

    def dynamicalModelTest(self):
        w = World()
        add_human36(w)
        w.update_dynamic()
        self.assertEqual(w.mass[5,5], 73.000000000000014)
        self.assertEqual(w.mass[41, 41], 0.10208399155688053) # neck
        self.assertEqual(w.mass[40,40], 0.020356790291165189) # neck
        self.assertEqual(w.mass[39, 39], 0.10430013572386694) # neck
        self.assertEqual(w.mass[16, 16], 0.0093741757009949949) # foot
        self.assertEqual(w.mass[17, 17], 0.001397215796713388) # foot
        self.assertEqual(w.mass[10, 10], 0.0093741757009949949) # foot
        self.assertEqual(w.mass[11, 11], 0.001397215796713388) # foot

ts = unittest.TestSuite()
ts.addTest(Human36TestCase('anatomicalLengthTest'))
ts.addTest(Human36TestCase('tagsTest'))
ts.addTest(Human36TestCase('test'))
