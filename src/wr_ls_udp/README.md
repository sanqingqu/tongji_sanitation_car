### How to drive the wr_lidar with ROS

After you installed the **ROS** environment, follow the following steps, you can drive the wr_2d_lidar.

**1st.** You need to configure your local static network ip address to make sure your device and the wr_lidar are in the same network segment. (The lidar's default ip is `192.168.0.10`)

**2nd.** After you configure your static ip address, you need to build the ros working space and source the env,    with following commands.

> cd Your_src_directory
>
> `catkin_make`
>
> `source devel/setup.bash`

**3rd**. Drive the wr_lidar with ros

> 	roslaunch wr_ls1207de_udp_with_1_lidar.launch（or wr_ls1207de_udp_with_2_lidars.launch） 

**4th.**  open another terminal 

> Source your ros-env
>
> `rviz`

