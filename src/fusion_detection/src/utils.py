#!/usr/bin/env python
#pylint: disable=no-member
import cv2
import numpy as np
import yaml
from scipy.spatial.transform import Rotation
from argparse import Namespace
from time import time

def cfg_extrinsic_type(extrinsic):
    extrinsic = Namespace(**yaml.safe_load(extrinsic))
    R = Rotation.from_euler(extrinsic.axis, extrinsic.degrees, degrees=True).as_dcm()
    t = np.array(extrinsic.translate).reshape((3, 1))
    return Namespace(R=R, t=t)

class ImageCropper:

    def __init__(self, camera_mat, new_camera_mat, frame_shape):
        self.map1, self.map2 = cv2.initUndistortRectifyMap(
            camera_mat, np.array([0.0, 0.0, 0.0, 0.0]), np.eye(3, 3), new_camera_mat, frame_shape, cv2.CV_16SC2)

    def __call__(self, frame):
        return cv2.remap(frame, self.map1, self.map2, cv2.INTER_LINEAR)

class BBoxReverter:

    def __init__(self, camera_mat, new_camera_mat):
        pass

    def __call__(self, bbox):
        pass

class Timer:

    def __init__(self):
        self.start = time()
        self.stop = None
        self.lowpass = 0.3
        self.highpass = 1 - self.lowpass
        self._fps = None
    
    def count(self):
        self.stop = time()
        current_fps = 1.0 / (self.stop - self.start + 1e-3)
        self.start = self.stop
        if self._fps is None:
            self._fps = current_fps
        else:
            self._fps = self._fps * self.lowpass + current_fps * self.highpass
    
    def fps(self):
        return self._fps

    def msec(self):
        if self._fps is None: return None
        return 1000.0 / self._fps

class MsgBuffer:

    def __init__(self, size=10):
        self.raw = []
        self.max_size = size
    
    def __len__(self): return len(self.raw)

    def __getitem__(self, i):
        if len(self.raw) == 0: return None
        retval = self.raw[i] 
        self.raw = self.raw[i+1:]
        return retval
    
    def append(self, new_msg):
        if len(self.raw) == self.max_size:
            del self.raw[0]
        self.raw.append(new_msg)
    
    def find_nearest(self, query_msg):
        if query_msg is None: return None
        min_diff_time, min_diff_idx = 1e5, -1
        query_stamp = query_msg.header.stamp
        for idx, msg in enumerate(self.raw):
            msg_stamp = msg.header.stamp
            diff_secs = query_stamp.secs - msg_stamp.secs
            diff_nsecs = query_stamp.nsecs - msg_stamp.nsecs
            diff_time = abs(diff_secs + diff_nsecs * 1e-9)
            if diff_time < min_diff_time:
                min_diff_time, min_diff_idx = diff_time, idx
        if min_diff_idx < 0: return None
        return self[min_diff_idx]

class AverageMeter(object):
    """Computes and stores the average and current value"""

    def __init__(self):
        self.reset()

    def reset(self):
        self.val = 0
        self.avg = 0
        self.sum = 0
        self.count = 0

    def update(self, val, n=1):
        self.val = val
        self.sum += val * n
        self.count += n
        self.avg = self.sum / self.count
