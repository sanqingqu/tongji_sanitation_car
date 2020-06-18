## Tongji sanitation-car project's visual perception repo

This repo is used to maintain the *tongji sanitation-car* project's visual perception  codes.

#### Env:

> [Ros melodic](http://wiki.ros.org/melodic/Installation/Ubuntu) + Ubuntu 18.04 + Python 2.7
>
> > Due to the requirements of the Ros, Python 2.7 is required.

#### Use Cameras

read fisheye camera image:
```bash
rosrun libuvc_camera camera_node _width:=1920 _height:=1080 _frame_rate:=30 _video_mode:=mjpeg # should not change the args
```

calibrate it
```bash
rosrun camera_calibration camera_calibrator_tj.py # will print params at end
```

show undistorted
```bash
rosrun camera_calibration camera_undistorted_tj_lower.py # should be modified by infomation printed by camera_calibrator_tj.py
```
