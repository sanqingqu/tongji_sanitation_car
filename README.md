## Tongji sanitation-car project's visual perception repo

This repo is used to maintain the *tongji sanitation-car* project's visual perception  codes.

#### Env:

> [Ros melodic](http://wiki.ros.org/melodic/Installation/Ubuntu) + Ubuntu 18.04 + Python 2.7
>
> > Due to the requirements of the Ros, Python 2.7 is required.

#### Use WR_LiDAR

you should first configure your local static network ip address  to make sure your device and the wr_lidar are in the same network segment. (The lidar's default ip is `192.168.0.10`)

run the wr_lidar:

```bash
roslaunch wr_ls1207de_udp_with_1_lidar.launch
```

#### Use Camera

it evolves changing the permission of accessing the cameras, see http://wiki.ros.org/libuvc_camera for details.

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

#### Use Image and Lidar Fusion

The `lidar_camera_fusion_detection` package evolves three scripts.

extract camera_input img.

```bash
rosrun camera_lidar_fusion_detectn img_extract.py
```

extract wr_lidar laser_scan msg.

```bash
rosrun camera_lidar_fusion_detectn laser_point_extract.py
```

fusion wr_lidar and input_undistorted img

```bash
rosrun camera_lidar_fusion_detection img_laser_fusion.py --debug_mode # use --help to see more detail configurations.
```

