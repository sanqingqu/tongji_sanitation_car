#!/usr/bin/env python
#pylint: disable=no-member
import cv2
import curses
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

def cfg_bbox_type(bbox):
    bbox = Namespace(**yaml.safe_load(bbox))
    return bbox

def valid_format(format_string, x, value_type, fallback_value):
    return (format_string % x) if isinstance(x, value_type) else fallback_value

def pdb(scr):
    curses.nocbreak()
    scr.keypad(0)
    curses.echo()
    curses.endwin()
    __import__('pdb').set_trace()

def argmin(a):
    return min(range(len(a)), key=lambda x: a[x])

def argmax(a):
    return max(range(len(a)), key=lambda x: a[x])

def bbox_distance(bbox1, bbox2):
    diff = np.array([
        bbox1.x_top_left - bbox2.x_top_left,
        bbox1.y_top_left - bbox2.y_top_left,
        bbox1.x_bottom_right - bbox2.x_bottom_right,
        bbox1.y_bottom_right - bbox2.y_bottom_right])
    return np.linalg.norm(diff)

class ImageCropper:

    def __init__(self, camera_mat, new_camera_mat, frame_shape):
        self.map1, self.map2 = cv2.initUndistortRectifyMap(
            camera_mat, np.array([0.0, 0.0, 0.0, 0.0]), np.eye(3, 3), new_camera_mat, frame_shape, cv2.CV_16SC2)

    def __call__(self, frame):
        return cv2.remap(frame, self.map1, self.map2, cv2.INTER_LINEAR)

class BBoxTransformer:

    def __init__(self, camera_mat, new_camera_mat):
        self.mat = np.matmul(new_camera_mat.reshape((3, 3)), np.linalg.inv(camera_mat.reshape((3, 3)))).reshape((1, 3, 3))

    def __call__(self, bbox):
        coord = np.array([[bbox.x_top_left, bbox.y_top_left, 1.], [bbox.x_bottom_right, bbox.y_bottom_right, 1.]]).reshape((2, 3, 1))
        coord = np.matmul(self.mat, coord)
        bbox.x_top_left, bbox.y_top_left, bbox.x_bottom_right, bbox.y_bottom_right = \
                coord[0, 0, 0], coord[0, 1, 0], coord[1, 0, 0], coord[1, 1, 0]
        return bbox

class Timer:

    def __init__(self, lowpass=0.1):
        self.start = time()
        self.stop = None
        self.lowpass = lowpass 
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

    def __enter__(self):
        self.start = time()

    def __exit__(self, exc_type, exc_value, exc_traceback):
        self.count()
    
    def fps(self):
        return self._fps

    def msec(self):
        if self._fps is None: return None
        return 1000.0 / self._fps

    def fmt_msec(self, fmt="%.2f"):
        return valid_format(fmt, self.msec(), float, "None")

    def fmt_fps(self, fmt="%.1f"):
        return valid_format(fmt, self.fps(), float, "None")

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

class MedianMeter:

    def __init__(self):
        self.reset()
    
    def reset(self):
        self.raw = []
    
    def update(self, val):
        self.raw.append(val)
    
    def value(self):
        return np.median(self.raw).item()