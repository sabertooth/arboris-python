# coding=utf-8

import unittest
from ArborisTests import BaseTest
from arboris.controllers import WeightController, ProportionalDerivativeController
from arboris.core import simplearm
from abc import ABCMeta, abstractmethod, abstractproperty
from arboris.core import NamedObject, Controller, World, Joint
from numpy import array, zeros, dot, ix_, ndarray
import arboris.homogeneousmatrix
from arboris.joints import LinearConfigurationSpaceJoint

class ControllersTestCase(BaseTest):
    def testConstructor(self):
        w1 = World()
        wc = WeightController()
        j = w1.getjoints()
        pdc = ProportionalDerivativeController(j)

    def testWeightController(self):
        w = simplearm()
        joints = w.getjoints()
        joints['Shoulder'].gpos[0] = 3.14/4
        joints['Elbow'].gpos[0] = 3.14/4
        joints['Wrist'].gpos[0] = 3.14/4
        c = WeightController()
        w.register(c)
        w.init()
        w.update_dynamic() #TODO change for update_kinematic
        self.assertListsAlmostEqual(c.update()[0],
        ([ 7.69376549,  2.49329922,  0.13889997]))
        self.assertListsAlmostEqual(c.update()[1],
        [[ 0.,  0.,  0.],[ 0.,  0.,  0.],[ 0.,  0.,  0.]])

    def testProportionalDerivativeController(self):
        w = simplearm()
        joints = w.getjoints()
        joints['Shoulder'].gpos[0] = 3.14/4
        joints['Elbow'].gpos[0] = 3.14/4
        joints['Wrist'].gpos[0] = 3.14/4
        pdc = ProportionalDerivativeController(joints,)
        w.register(pdc)
        w.init()
        w.update_dynamic()
        self.assertListsAlmostEqual(pdc.update(1/2)[0],([ 0.,  0.,  0.]))
        self.assertListsAlmostEqual(pdc.update(1/2)[1],
        [[ 0.,  0.,  0.],[ 0.,  0.,  0.],[ 0.,  0.,  0.]]) 

ts = unittest.TestSuite()
ts.addTest(ControllersTestCase('testConstructor'))
ts.addTest(ControllersTestCase('testWeightController'))
ts.addTest(ControllersTestCase('testProportionalDerivativeController'))