#!/usr/bin/env python

""" Show 3D point cloud
    Author: Huanle Zhang
    Website: www.huanlezhang.com
"""

import numpy as np
import pptk


def show_point_cloud(pc_data):
    # pc_data: point cloud data
    pptk.viewer(pc_data[:, :3])


if __name__ == '__main__':
    #path_to_point_cloud = '/home/johny/CustomDataset/training/velodyne/000000.bin'
    #path_to_point_cloud = '/home/johny/Kitti/gt_database/0_Pedestrian_0.bin'
    path_to_point_cloud = '/home/johny/CustomDataset/gt_database/0_Pedestrian_0.bin'

    point_cloud_data = np.fromfile(path_to_point_cloud, '<f4')  # little-endian float32
    point_cloud_data = np.reshape(point_cloud_data, (-1, 4))  # x, y, z, r
    show_point_cloud(point_cloud_data)