#!/usr/bin/env python
#pylint: disable=no-member

import numpy as np
import cv2
import screeninfo
import easydict

class Visualizer:

    def __init__(self, screen_id = 0, window_name="TJSAN", full_screen=False):

        self.screen_id = screen_id
        self.window_name = window_name
        self.screen = screeninfo.get_monitors()[self.screen_id]

        if full_screen:
            cv2.namedWindow(self.window_name, cv2.WND_PROP_FULLSCREEN)
            cv2.moveWindow(self.window_name, self.screen.x - 1, self.screen.y - 1)
            cv2.setWindowProperty(self.window_name, cv2.WND_PROP_FULLSCREEN,
                    cv2.WINDOW_FULLSCREEN)
        
        self.states = easydict.EasyDict()
        self.exited = False
    
    def update(self, **kwargs):

        for k, v in kwargs.items():
            self.states[k] = v
        
        self.render()
        
    def render(self):
        if self.exited: return

        frame = self.states.lower_img_raw

        cv2.imshow(self.window_name, frame)

        self.waitKey()
        
    def waitKey(self, timeout=1):
        key = cv2.waitKey(timeout)
        if 27 == key:
            cv2.destroyAllWindows()
            self.exited = True
