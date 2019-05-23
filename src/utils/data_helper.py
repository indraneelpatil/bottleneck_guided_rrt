#!/usr/bin/python3.5
# -*- coding: utf-8 -*-

""" Data, including point cloud and ground truth, preprocess helper function """
# TODO KiTTI data helper

import numpy as np
#import pcl
import sys
import os
import glob

# Sydney Urban Objects LiDAR dataset formats
fields = ['t', 'intensity', 'id',
          'x', 'y', 'z',
          'azimuth', 'range', 'pid']
types = ['int64', 'uint8', 'uint8',
         'float32', 'float32', 'float32',
         'float32', 'float32', 'int32']

# label dict for Sydney Urban Object Dataset, ref:http://www.acfr.usyd.edu.au/papers/SydneyUrbanObjectsDataset.shtml
SUOD_label_dictionary = {
    '4wd': 0, 'building': 1, 'bus': 2, 'car': 3, 'pedestrian': 4, 'pillar': 5, 'pole': 6,
    'traffic_lights': 7, 'traffic_sign': 8, 'tree': 9, 'truck': 10, 'trunk': 11, 'ute': 12, 'van': 13
}
SUOD_label_dictionary_rev = {
    0: '4wd',            1: 'building',     2: 'bus',  3: 'car',    4: 'pedestrian', 5: 'pillar', 6: 'pole',
    7: 'traffic_lights', 8: 'traffic_sign', 9: 'tree', 10: 'truck', 11: 'trunk',     12: 'ute',   13: 'van'
}

OCCUPIED = 1
FREE = 0

def load_points_from_bin(bin_file, with_intensity=False):
    """

    :param bin_file:
    :param with_intensity:
    :return: (N, 3) or (N, 4)
    """

    binType = np.dtype(dict(names=fields, formats=types))
    data = np.fromfile(bin_file, binType)

    # 3D points, one per row
    if with_intensity:
        points = np.vstack([data['x'], data['y'], data['z'], data['intensity']]).T
    else:
        points = np.vstack([data['x'], data['y'], data['z']]).T

    return points


def get_SUOD_label(index):
    return SUOD_label_dictionary_rev[index]


def save_pcd_from_bin(bin_file, with_intensity=False):
    """
    Read bins from `fold` as x,y,z and convert into `*.pcd`

    Args:
    `fold` the path of fold*.txt that need to convert into pcd

    """

    points = load_points_from_bin(bin_file, with_intensity)

    cloud = pcl.PointCloud(np.float32(points))
    pcl.save(cloud, '{}.pcd'.format(bin_file.split('.bin')[0]))


def voxelize(points, voxel_size=(24, 24, 24), padding_size=(32, 32, 32), resolution=0.1):
    """
    Convert `points` to centerlized voxel with size `voxel_size` and `resolution`, then padding zero to
    `padding_to_size`. The outside part is cut, rather than scaling the points.

    Args:
    `points`: pointcloud in 3D numpy.ndarray
    `voxel_size`: the centerlized voxel size, default (24,24,24)
    `padding_to_size`: the size after zero-padding, default (32,32,32)
    `resolution`: the resolution of voxel, in meters

    Ret:
    `voxel`:32*32*32 voxel occupany grid
    `inside_box_points`:pointcloud inside voxel grid
    """

    if abs(resolution) < sys.float_info.epsilon:
        print('error input, resolution should not be zero')
        return None, None

    # remove all non-numeric elements of the said array
    points = points[np.logical_not(np.isnan(points).any(axis=1))]

    # filter outside voxel_box by using passthrough filter
    # TODO Origin, better use centroid?
    origin = (np.min(points[:, 0]), np.min(points[:, 1]), np.min(points[:, 2]))
    # set the nearest point as (0,0,0)
    points[:, 0] -= origin[0]
    points[:, 1] -= origin[1]
    points[:, 2] -= origin[2]
    # logical condition index
    x_logical = np.logical_and((points[:, 0] < voxel_size[0] * resolution), (points[:, 0] >= 0))
    y_logical = np.logical_and((points[:, 1] < voxel_size[1] * resolution), (points[:, 1] >= 0))
    z_logical = np.logical_and((points[:, 2] < voxel_size[2] * resolution), (points[:, 2] >= 0))
    xyz_logical = np.logical_and(x_logical, np.logical_and(y_logical, z_logical))
    inside_box_points = points[xyz_logical]

    # init voxel grid with zero padding_to_size=(32*32*32) and set the occupany grid
    voxels = np.zeros(padding_size)
    # centerlize to padding box
    center_points = inside_box_points + (padding_size[0] - voxel_size[0]) * resolution / 2
    # TODO currently just use the binary hit grid
    x_idx = (center_points[:, 0] / resolution).astype(int)
    y_idx = (center_points[:, 1] / resolution).astype(int)
    z_idx = (center_points[:, 2] / resolution).astype(int)
    voxels[x_idx, y_idx, z_idx] = OCCUPIED

    return voxels, inside_box_points


def point_transform(points, tx, ty, tz, rx=0, ry=0, rz=0):
    """
      P(x, y, z) transform operation with translation(tx, ty, tz) and rotation(rx, ry, rz)
    :param original points: (N, 3)
    :param tx/y/z: in meter
    :param rx/y/z: in radian
    :return: transformed points: (N, 3)
    """

    N = points.shape[0]
    points = np.hstack([points, np.ones((N, 1))])

    mat1 = np.eye(4)
    mat1[3, 0:3] = tx, ty, tz
    points = np.matmul(points, mat1)

    if rx != 0:
        mat = np.zeros((4, 4))
        mat[0, 0] = 1
        mat[3, 3] = 1
        mat[1, 1] = np.cos(rx)
        mat[1, 2] = -np.sin(rx)
        mat[2, 1] = np.sin(rx)
        mat[2, 2] = np.cos(rx)
        points = np.matmul(points, mat)

    if ry != 0:
        mat = np.zeros((4, 4))
        mat[1, 1] = 1
        mat[3, 3] = 1
        mat[0, 0] = np.cos(ry)
        mat[0, 2] = np.sin(ry)
        mat[2, 0] = -np.sin(ry)
        mat[2, 2] = np.cos(ry)
        points = np.matmul(points, mat)

    if rz != 0:
        mat = np.zeros((4, 4))
        mat[2, 2] = 1
        mat[3, 3] = 1
        mat[0, 0] = np.cos(rz)
        mat[0, 1] = -np.sin(rz)
        mat[1, 0] = np.sin(rz)
        mat[1, 1] = np.cos(rz)
        points = np.matmul(points, mat)

    return points[:, 0:3]


def aug_data(points, aug_size, uniform_rotate_only=False):
    """
    Object segments data augmentation, translation as well as rotation refer to VoxelNet

    :param points:
    :param aug_size: creating n copies of each input instance, n is 12 or 18 refer VoxNet.
    :param uniform_rotate_only: just uniform rotate by "2π/aug_size"
    :return:
    """
    np.random.seed()
    rot_interval = 2 * np.pi / (aug_size+1)

    points_list = [points]
    for idx in range(1, aug_size+1):
        # rotate by a uniformally distributed random variable
        r_z = np.random.uniform(-np.pi / 10, np.pi / 10)
        t_x = np.random.normal()
        t_y = np.random.normal()
        t_z = np.random.normal()

        if uniform_rotate_only:
            # creating n copies of each input instance, each rotated 360◦/n intervals around the z axis.
            r_z = rot_interval * idx
            t_x = t_y = t_z = 0.

        # translation and rotation
        points_list.append(point_transform(points, t_x, t_y, t_z, rz=r_z))

    return np.float32(points_list)


def load_data_from_npy(npy_dir, mode='training', type='dense'):
    """
    Get all voxels and corresponding labels from preprocess `npy_dir`.

    Args:
    `npy_dir`: path to the folder that contains `training` and `testing`.
    `mode`: folder that contains all the `npy` datasets
        training
        testing
    `type`: type of npy for future use in sparse tensor, values={`dense`,`sparse`}

    Ret:
    `voxels`: list of voxel grids ()
    `labels`: list of labels
    """

    input_path = os.path.join(npy_dir, mode, '*.npy')

    # TODO:(vincent.cheung.mcer@gmail.com) not yet add support for multiresolution npy data
    # TODO:(vincent.cheung.mcer@gmail.com) not yet support sparse npy
    voxels_list = []
    label_list = []
    for npy_f in glob.iglob(input_path):
        # extract the label from path+file_name: e.g.`./voxel_npy/pillar.2.3582_12.npy`
        file_name = npy_f.split('/')[-1]
        label = SUOD_label_dictionary[file_name.split('.')[0]]
        label_list.append(label)

        # load *.npy
        voxels = np.load(npy_f).astype(np.float32)
        voxels_list.append(voxels)


    return np.array(voxels_list), np.array(label_list)

def load_labelled_data():

    DATA_input='/home/indraneel/tf-demo/VoxNet/labelled_data/inputs'
    DATA_output='/home/indraneel/tf-demo/VoxNet/labelled_data/outputs'
    DATA_endpoints='/home/indraneel/tf-demo/VoxNet/labelled_data/start_end_points'
    input_path = os.path.join(DATA_input, '*.npy')
    output_path = os.path.join(DATA_output, '*.npy')
    endpoints_path = os.path.join(DATA_endpoints, '*.npy')

    voxels_list = []
    label_list = []
    endpoints_list = []
    for npy_f in glob.iglob(input_path):
        file_name=npy_f.split('/')[-1]
        #print(file_name)
        voxels=np.load(npy_f).astype(np.float32)
        voxels_list.append(voxels)
        for npy_file in glob.iglob(output_path):
            file_name2=npy_file.split('/')[-1]
            if file_name==file_name2:
                #print("************",file_name2)
                labels=np.load(npy_file).astype(np.float32)
                label_list.append(labels)

        for npyfile in glob.iglob(endpoints_path):
            file_name3=npy_file.split('/')[-1]
            if file_name==file_name3:
                #print("************",file_name2)
                endpoints=np.load(npyfile).astype(np.float32)
                endpoints_list.append(endpoints)

    return np.array(voxels_list), np.array(label_list), np.array(endpoints_list)





