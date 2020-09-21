#!/usr/bin/env python
#pylint: disable=no-member

import numpy as np
import cv2
import screeninfo
import easydict

class Visualizer:

    CANDIDATE_TRASHBIN_COLOR = (0, 165, 255) # orange
    BESTVALID_TRASHBIN_COLOR = (0, 255, 0)   # green
    ROI_BOUNDING_BOX_COLOR = (0, 0, 255) # red

    def __init__(self, screen_id = 0, window_name="TJSAN", full_screen=False, roi=None):

        self.screen_id = screen_id
        self.window_name = window_name
        self.screen = screeninfo.get_monitors()[self.screen_id]
        self.roi = self._extract_bbox(roi) if roi else None

        if full_screen:
            cv2.namedWindow(self.window_name, cv2.WND_PROP_FULLSCREEN)
            cv2.moveWindow(self.window_name, self.screen.x - 1, self.screen.y - 1)
            cv2.setWindowProperty(self.window_name, cv2.WND_PROP_FULLSCREEN,
                    cv2.WINDOW_FULLSCREEN)
        
        self.states = easydict.EasyDict()
        self.exited = False
    
    def update(self, **kwargs):

        for k, v in kwargs.items():
            if v is None: continue
            self.states[k] = v
        
        self.render()
        
    def render(self, show_image_bin_points=True):
        if self.exited: return

        if 'lower_img_raw' in self.states:
            frame = self.states.lower_img_raw
        else:
            frame = np.ones((1920, 1080, 3), dtype=np.uint8) * 127
        
        if 'detect_results' in self.states:
            for bbox in self.states.detect_results.cand_trash_boxes:
                top_left, bottom_right = self._extract_bbox(bbox)
                cv2.rectangle(frame, top_left, bottom_right, self.CANDIDATE_TRASHBIN_COLOR, 2)
            best_valid_trashbin = self.states.detect_results.best_valid_trashbin
            if best_valid_trashbin is not None:
                top_left, bottom_right = self._extract_bbox(best_valid_trashbin)
                cv2.rectangle(frame, top_left, bottom_right, self.BESTVALID_TRASHBIN_COLOR, 2)
            if show_image_bin_points:
                image_bin_points = self.states.detect_results.image_bin_points
                if image_bin_points is not None:
                    for point in image_bin_points:
                        cv2.circle(frame, (point[0], point[1]), 6, (0, 0, 255), -1)
            if self.roi is not None:
                top_left, bottom_right = self.roi
                cv2.rectangle(frame, top_left, bottom_right, self.ROI_BOUNDING_BOX_COLOR, 2)

            if 'upper_img_raw' in self.states and self.states.upper_img_raw is not None:
                for bbox in self.states.detect_results.cand_human_boxes:
                    top_left, bottom_right = self._extract_bbox(bbox)
                    cv2.rectangle(self.states.upper_img_raw, top_left, bottom_right, self.CANDIDATE_TRASHBIN_COLOR, 2)

        if 'upper_img_raw' in self.states and self.states.upper_img_raw is not None:
            upper_img = cv2.resize(self.states.upper_img_raw, None, fx=0.25, fy=0.25)
            frame[:upper_img.shape[0]:, -upper_img.shape[1]:] = upper_img

        cv2.imshow(self.window_name, frame)
        self.waitKey()
        
    def waitKey(self, timeout=1):
        key = cv2.waitKey(timeout)
        if 27 == key:
            cv2.destroyAllWindows()
            self.exited = True

    def _extract_bbox(self, bbox):
        top_left = (int(round(bbox.x_top_left)), int(round(bbox.y_top_left)))
        bottom_right = (int(round(bbox.x_bottom_right)),int(round(bbox.y_bottom_right)))
        return top_left, bottom_right
