#!/usr/bin/python2
# -*- coding: utf-8 -*-

""" Functions for data visualization. """

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def plot3DVoxel(voxels):
    fig = plt.figure('Point Cloud 3D Voxelization')
    plt3d = fig.gca(projection='3d')

    occupied = (voxels == 1)
    free = (voxels == 0)

    # # set the colors of each object
    colors = np.zeros(voxels.shape + (4,))
    colors[free] = [0.1, 0.1, 0.1, 0.1]
    colors[occupied] = [0.8, 0.8, 0.8, 1.0]

    # setting camera angle
    plt3d.set_xlabel('X', fontsize=9)
    plt3d.set_ylabel('Y', fontsize=9)
    plt3d.set_zlabel('Z', fontsize=9)
    plt3d.set_xlim3d(0, voxels.shape[0])
    plt3d.set_ylim3d(0, voxels.shape[1])
    plt3d.set_zlim3d(0, voxels.shape[2])

    # and plot everything
    # plt3d.voxels(occupied | free, facecolors=colors, edgecolor='k', linewidth=0.8)
    plt3d.voxels(occupied, facecolors=colors, edgecolor='k', linewidth=0.8)

    dx, dy, dz = voxels.shape
    x = y= z = 0
    xx = [x, x, x+dx, x+dx, x]
    yy = [y, y+dy, y+dy, y, y]
    kwargs = {'alpha': 1, 'color': 'red'}
    plt3d.plot3D(xx, yy, [z]*5, **kwargs)
    plt3d.plot3D(xx, yy, [z+dz]*5, **kwargs)
    plt3d.plot3D([x, x], [y, y], [z, z+dz], **kwargs)
    plt3d.plot3D([x, x], [y+dy, y+dy], [z, z+dz], **kwargs)
    plt3d.plot3D([x+dx, x+dx], [y+dy, y+dy], [z, z+dz], **kwargs)
    plt3d.plot3D([x+dx, x+dx], [y, y], [z, z+dz], **kwargs)

    plt.show()