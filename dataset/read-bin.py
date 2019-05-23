# -*- coding: utf-8 -*-
""" Simple example for loading object binary data. """
import numpy as np

names = ['t','intensity','id',
         'x','y','z',
         'azimuth','range','pid']

formats = ['int64', 'uint8', 'uint8',
           'float32', 'float32', 'float32',
           'float32', 'float32', 'int32']

binType = np.dtype( dict(names=names, formats=formats) )
data = np.fromfile('objects/excavator.0.10974.bin', binType)

# 3D points, one per row
P = np.vstack([ data['x'], data['y'], data['z'] ]).T

