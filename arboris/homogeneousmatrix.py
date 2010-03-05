# coding=utf-8
"""Functions for working with homogeneous matrices."""

__author__ = ("Sébastien BARTHÉLEMY <barthelemy@crans.org>")

from numpy import array, zeros, sin, cos, dot, hstack, vstack
import numpy

tol=1e-9

def transl(t_x, t_y, t_z):
    """Homogeneous matrix of a translation.

    INPUT: ``t_x, t_y, t_z`` - coordinates of the translation vector in 3d space

    OUTPUT: Homogeneous matrix of the translation
    """
    return array(
        [[ 1. , 0., 0., t_x],
         [ 0. , 1., 0., t_y],
         [ 0. , 0., 1., t_z],
         [ 0.,  0., 0., 1.]])


def rotzyx(angle_z, angle_y, angle_x):
    """homogeneous transformation matrix from pitch-roll-yaw angles
    
    In short:  R = Rz * Ry * Rx
    """
    
    sz = sin(angle_z)
    cz = cos(angle_z)
    sy = sin(angle_y)
    cy = cos(angle_y)
    sx = sin(angle_x)
    cx = cos(angle_x)
    return array(
        [[ cz*cy, cz*sy*sx-sz*cx, cz*sy*cx+sz*sx, 0.],
         [ sz*cy, sz*sy*sx+cz*cx, sz*sy*cx-cz*sx, 0.],
         [-sy   , cy*sx         , cy*cx         , 0.],
         [ 0.   , 0.            , 0.            , 1.]])
         
         
def rotzy(angle_z, angle_y):
    """homogeneous transformation matrix from pitch-roll-yaw angles)
    
    In short:  R = Rz * Ry

    """
    sz = sin(angle_z)
    cz = cos(angle_z)
    sy = sin(angle_y)
    cy = cos(angle_y)
    return array(
        [[ cz*cy,-sz, cz*sy, 0.],
         [ sz*cy, cz, sz*sy, 0.],
         [-sy   , 0., cy   , 0.],
         [ 0.   , 0., 0.   , 1.]])


def rotzx(angle_z, angle_x):
    """homogeneous transformation matrix from pitch-roll-yaw angles)
    
    In short:  R = Rz * Rx

    """
    sz = sin(angle_z)
    cz = cos(angle_z)
    sx = sin(angle_x)
    cx = cos(angle_x)
    return array(
        [[ cz,-sz*cx, sz*sx, 0.],
         [ sz, cz*cx,-cz*sx, 0.],
         [ 0., sx   , cx   , 0.],
         [ 0., 0.   , 0.   , 1.]])
         
         
def rotyx(angle_y, angle_x):
    """homogeneous transformation matrix from pitch-roll-yaw angles)
    
    In short:  R = Ry * Rx

    """
    sy = sin(angle_y)
    cy = cos(angle_y)
    sx = sin(angle_x)
    cx = cos(angle_x)
    return array(
        [[ cy, sy*sx, sy*cx, 0.],
         [ 0., cx   ,-sx   , 0.],
         [-sy, cy*sx, cy*cx, 0.],
         [ 0., 0.   , 0.   , 1.]])

def rotx(angle):
    """
    Homogeneous matrix of a rotation around the x-axis

    """
    ca = cos(angle)
    sa = sin(angle)
    H = array(
        [[1,  0,   0,  0],
         [0, ca, -sa,  0],
         [0, sa,  ca,  0],
         [0,  0,   0,  1]])
    return H

def roty(angle):
    """
    Homogeneous matrix of a rotation around the y-axis

    """
    ca = cos(angle)
    sa = sin(angle)
    H = array(
        [[ ca,  0,  sa,  0],
         [  0,  1,   0,  0],
         [-sa,  0,  ca,  0],
         [  0,  0,   0,  1]])
    return H

def rotz(angle):
    """
    Rotation around the z-axis

    """
    ca = cos(angle)
    sa = sin(angle)
    H = array(
        [[ca, -sa, 0, 0],
         [sa,  ca, 0, 0],
         [ 0,   0, 1, 0],
         [ 0,   0, 0, 1]])
    return H

def ishomogeneousmatrix(H, tol=tol):
    """
    Return true if input is an homogeneous matrix
    """
    return (H.shape == (4,4)) \
        and (numpy.abs(numpy.linalg.det(H[0:3,0:3])-1)<=tol) \
        and (H[3,0:4]==[0,0,0,1]).all()

def pdot(H, point):
    """Frame change for a point.
    """
    assert ishomogeneousmatrix(H)
    return dot(H[0:3,0:3], point) + H[0:3, 3]

def vdot(H, vec):
    """Frame change for a vector.
    """
    assert ishomogeneousmatrix(H)
    return dot(H[0:3,0:3], vec)

def inv(H):
    """
    Invert an homogeneous matrix.

    """
    assert ishomogeneousmatrix(H)
    R = H[0:3,0:3]
    p = H[0:3,3:4]
    return vstack( (hstack((R.T,-dot(R.T,p))), [0,0,0,1]))

def adjoint(H):
    """
    Adjoint (6x6 matrix) of the homogeneous matrix.

    :param H: homogeneous matrix 
    :type H: 4x4 ndarray
    :return: adjoint matrix
    :rtype: 6x6 ndarray

    """
    assert ishomogeneousmatrix(H), H
    R = H[0:3,0:3]
    p = H[0:3,3:4]
    pxR = dot(
        array(
            [[    0, -p[2],  p[1]],
             [ p[2],     0, -p[0]],
             [-p[1],  p[0],     0]]),
        R)
    return vstack((
        hstack((R  , zeros((3,3)))),
        hstack((pxR, R))))

def iadjoint(H):
    """
    Return the adjoint (6x6 matrix) of the inverse homogeneous matrix.
    """
    return adjoint(inv(H))

