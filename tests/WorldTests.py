# coding: utf-8

import unittest
from ArborisTests import BaseTest
from arboris.core import Body, SubFrame, World
from arboris.homogeneousmatrix import transl
from arboris.joints import RyJoint, RzRxJoint, RyRxJoint, RzRyRxJoint
from arboris.robots.simplearm import add_simplearm
from numpy import eye, zeros


class WorldTestCase(BaseTest):

    def testConstructor(self):
        w = World('ArborisLand')


class TestLinks(WorldTestCase):

    def testLinksCreation(self):
        w = World()
        bulb = RyRxJoint(name='bulb')
        stem = Body(name='stem', mass=eye(6)*0.1)
        w.add_link(w.ground, bulb, stem)

        stem_top = SubFrame(stem, bpose=transl(0, 0, 1), name='stem_top')
        flowerR = RyJoint(gpos=.5, name='flowerR')
        leafR = Body('leafR', mass=eye(6)*0.01)
        w.add_link(stem_top, flowerR, leafR)

        flowerL = RyJoint(gpos=(-.5), name='flowerL')
        leafL = Body('leafL', mass=eye(6)*.01)
        w.add_link(stem_top, flowerL, leafL)

        frames = [ w.ground, stem, leafR, leafL, stem_top ]
        for frame1, frame2 in zip(w.iterframes(), frames):
            self.assertEqual(frame1, frame2)

        joints = [ bulb, flowerR, flowerL ]
        for joint1, joint2 in zip(w.iterjoints(), joints):
            self.assertEqual(joint1, joint2)

    def testLinksReplacement(self):
        w = World()
        bulb = RyRxJoint(name='bulb')
        stem = Body(name='stem', mass=eye(6)*0.1)
        w.add_link(w.ground, bulb, stem)

        new_bulb = RzRxJoint(name='bulb')
        w.replace_joint(bulb, new_bulb) # By another joint
        joints = w.getjoints()
        self.assertEqual(len(joints), 1)
        self.assertEqual(joints[0], new_bulb)

        new_new_bulb = RzRyRxJoint(name='bulb')
        w.replace_joint(new_bulb, w.ground, new_new_bulb, stem) # By a kinematic chain
        joints = w.getjoints()
        self.assertEqual(len(joints), 1)
        self.assertEqual(joints[0], new_new_bulb)


class TestUpdates(WorldTestCase):

    def testGeometricUpdate(self):
        w = World()
        add_simplearm(w)
        bodies = w.getbodies()
        arm = bodies['Arm']
        forearm = bodies['ForeArm']
        hand = bodies['Hand']
        w.update_geometric()
        self.assertListsAlmostEqual(arm.pose, eye(4))
        self.assertListsAlmostEqual(forearm.pose, [ [ 1. ,  0. ,  0. ,  0. ],
                                                    [ 0. ,  1. ,  0. ,  0.5],
                                                    [ 0. ,  0. ,  1. ,  0. ],
                                                    [ 0. ,  0. ,  0. ,  1. ] ])
        self.assertListsAlmostEqual(hand.pose, [ [ 1. ,  0. ,  0. ,  0. ],
                                                 [ 0. ,  1. ,  0. ,  0.9],
                                                 [ 0. ,  0. ,  1. ,  0. ],
                                                 [ 0. ,  0. ,  0. ,  1. ] ])

ts = unittest.TestSuite()
ts.addTest(WorldTestCase('testConstructor'))
ts.addTest(TestLinks('testLinksCreation'))
ts.addTest(TestLinks('testLinksReplacement'))
ts.addTest(TestUpdates('testGeometricUpdate'))