#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import numpy as np

pc_num = 0

class RecordPose:
    def __init__(self):
        self.timeStamp = None
        self.count = 0

    def main(self):
        # ROS node initialize
        rospy.init_node('bag2bin')
        rospy.Subscriber('/rslidar_points', PointCloud2, self.callback)
        rospy.spin()

    def callback(self, msg):
        cloud_points = list(pc2.read_points(msg, field_names = ("x", "y", "z", "intensity"), skip_nans=True))
        pc_arr = np.array(cloud_points).flatten()
        # Dividing by param_intensity_range to get intensity in desired range
        pc_arr[3::4] = 0
        global pc_num
        output_file = "/home/johny/CustomDatasetV2/testing/velodyne/{:06d}.bin".format(pc_num)
        pc_arr.astype('float32').tofile(output_file)
        pc_num += 1


if __name__ == '__main__':
    record_pose = RecordPose()
    record_pose.main()
