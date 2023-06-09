import scipy.io
import numpy as np
import pandas as pd

CALIB_FILE_PATH = '/home/johny/CustomDatasetV2/training/calib/000001.txt'
MAT_FILE_PATH   = '/home/johny/catkin_ws/src/tools/src/labels_eval.mat'
LABEL_OUT_PATH  = '/home/johny/CustomDatasetV2/training/label_2/'
_LIDAR_HEIGHT   = -0.80
FILE_OFFSET     = 145

class LabelConverter():
    def __init__(self, calib_path, mat_path, label_path):
        self.read_calib_file(calib_path)
        self.labels = self.from_matlab(mat_path)
        self.create_kitti_labels(self.labels, label_path)
        self.display()

    def display(self):
        print(self.labels)

    def read_calib_file(self, calib_path):
        """ Read in a calibration file and parse into a dictionary.
        Ref: https://github.com/utiasSTARS/pykitti/blob/master/pykitti/utils.py
        """
        data = {}
        with open(calib_path, "r") as f:
            for line in f.readlines():
                line = line.rstrip()
                if len(line) == 0:
                    continue
                key, value = line.split(":", 1)
                # The only non-float values in these files are dates, which
                # we don't care about anyway
                try:
                    data[key] = np.array([float(x) for x in value.split()])
                except ValueError:
                    pass
        
        self.V2C = data["Tr_velo_to_cam"]
        self.V2C = np.reshape(self.V2C, [3, 4])
        self.R0 = data["R0_rect"]
        self.R0 = np.reshape(self.R0, [3, 3])

    def get_dimensions(self, dimensions):
        return np.asarray([dimensions[2], dimensions[0], dimensions[1]])
    
    def lidar_to_cam(self, pts_3d_velo):
        # Using Kitti calibration file helps us to achieve the following code
        # --------- BEGIN ----------
        # m = np.mat([[0, -1, 0], 
        #             [0, 0, -1], 
        #             [1, 0, 0], ])
        
        # r = m * np.mat(pts_3d_velo).T
        # r = r.T
        # return np.asarray([r.tolist()[0][0], 
        #                    r.tolist()[0][1], 
        #                    r.tolist()[0][2]])
        # --------- END ----------
        def cart2hom(pts_3d):
            """ Input: nx3 points in Cartesian
                Oupput: nx4 points in Homogeneous by pending 1
            """
            pts_3d_hom = np.hstack((pts_3d, np.ones((1))))
            return pts_3d_hom
        # Velodyne to reference camera
        pts_3d_velo[2] = _LIDAR_HEIGHT
        pts_3d_velo = cart2hom(pts_3d_velo)  # nx4
        pts_3d_ref = np.dot(pts_3d_velo, np.transpose(self.V2C))
        # Reference camera to left camera
        return np.transpose(np.dot(self.R0, np.transpose(pts_3d_ref))).round(decimals=3)

    def from_matlab(self, path):
        # MatLab labels looks like this: ['xctr', 'yctr', 'zctr', 
        #                                 'xlen', 'ylen', 'zlen', 
        #                                 'xrot', 'yrot', 'zrot']
        mat     = scipy.io.loadmat(path)
        labels  = mat['labels'].astype(np.ndarray)
        df      = pd.DataFrame(columns = ['File', 'Label', 'Type', 'Truncated', 
                                          'Occluded', 'Alpha', 'BBox', 'Dimensions', 
                                          'Location', 'Rotation'])
        for file_counter, file in enumerate(labels):
            for label_counter, label in enumerate(file[0].round(decimals=3)):
                bbox2d      = np.asarray([0, 0, 50, 50])
                dimensions  = self.get_dimensions(np.asarray([label[3], label[4], label[5]]))
                location    = self.lidar_to_cam(np.asarray([label[0], label[1], label[2]]))
                new_label   = [file_counter, label_counter, 'Pedestrian', 
                               0.0, 0, 0, bbox2d, dimensions, location, 0]
                df.loc[len(df)] = new_label
        return df.set_index(['File', 'Label'])

    def create_kitti_labels(self, df, label_path):
        for file in range(len(df.groupby(level=0))):
            file_data = df.loc[[file]]
            with open(label_path + '{:06d}.txt'.format(file+FILE_OFFSET), 'x') as txtfile:
                for label in range(len(file_data.groupby(level=1))):
                    label_data = file_data.loc[file, label]
                    def parse(tag):
                        return ' '.join(str(val) for val in label_data[tag])
                    txtfile.write(f"{label_data['Type']} {label_data['Truncated']} {label_data['Occluded']} " \
                                  f"{label_data['Alpha']} {parse('BBox')} {parse('Dimensions')} " \
                                  f"{parse('Location')} {label_data['Rotation']}\n") \

if __name__ == '__main__':
    lc = LabelConverter(CALIB_FILE_PATH, MAT_FILE_PATH, LABEL_OUT_PATH)