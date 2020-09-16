#!/usr/bin/env python
#pylint: disable=no-member
import cv2
import rospy
import numpy as np

class SpeedEstimation:

    def __init__(self, image_row_min=200, downsample_step=5, lowpass_filter=0.5):
        self.previous_frame = None
        self.previous_frame_tick = None
        self.current_frame = None
        self.current_frame_tick = None
        self.flow = None
        self.current_speed = None
        self.filtered_speed = None
        self.lowpass_filter = lowpass_filter
        self.highpass_filter = 1 - lowpass_filter
        self.image_row_min = image_row_min
        self.downsample_step = downsample_step

    def __call__(self, new_frame=None, new_time=None):
        # get a new image frame to update (optionally) and return current speed
        if new_frame is None: return self.filtered_speed
        if new_time is None: new_time = rospy.get_time()
        new_frame = self._preprocess(new_frame)
        self.previous_frame, self.previous_frame_tick = self.current_frame, self.current_frame_tick
        self.current_frame, self.current_frame_tick = new_frame, new_time
        if self.previous_frame is None or self.current_frame_tick is None: return None
        _farneback_flags = cv2.OPTFLOW_USE_INITIAL_FLOW if self.flow is not None else 0
        # cv2.calcOpticalFlowFarneback(prev, next, flow, pyr_scale, levels, winsize, iterations, poly_n, poly_sigma, flags)
        self.flow = cv2.calcOpticalFlowFarneback(self.previous_frame, self.current_frame, self.flow, 0.5, 3, 15, 3, 5, 1.2, _farneback_flags)
        self.current_speed = self.flow[..., 0].flatten().mean() / (self.current_frame_tick - self.previous_frame_tick + 1e-3)
        self.filtered_speed = (self.lowpass_filter * self.filtered_speed + self.highpass_filter * self.current_speed) \
            if self.filtered_speed is not None else self.current_speed
        return self.filtered_speed

    def _preprocess(self, frame):
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        frame = frame[self.image_row_min::self.downsample_step, ::self.downsample_step]
        return frame
