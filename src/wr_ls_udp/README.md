#wr_ls_udp
####################
#### HOW TO RUN ####
# 1: build:
	catkin_make
# 2: install:
	catkin_make install
# 3.1: run with node:
	roslaunch wr_ls1207de_udp_with_1_lidar.launch（or wr_ls1207de_udp_with_2_lidars.launch） 
# 3.2: run with nodelet:
	roslaunch wr_ls_udp wr_ls1207de_udp_nodelet_with_1_lidar.launch (or wr_ls1207de_udp_nodelet_with_1_lidar.launch)
#       NOTE:wr_laser_manager nodelet will be launch by default, you can specify 
#	     the nodelet manager what you want by setting arguments "bringup_self_manager" and
#	     "manager_name" in the launch file.

####################
#### HOW TO VIEW FROM RVIZ ###
# 1: Run the package as introduced .
# 2: Start another terminal.
# 3: run rviz:
	rosrun rviz rviz
# 4: In the popup window, click [Add] on the left-bottum
# 5: In the popup window, select  [By topic] tab.
# 6: Select [LaserScan], then [OK]
# 7: Back to the main window, in the left operation panel:
     [Displays]->[Global Options]->[Fixed Frame]->Change the value of this item to [laser]
# 8: Then, you should be able to see the result in display window.

#################### 
#### HOW TO DISABLE DEBUG MODE ####
# 1: In cfg/WrLs.cfg
# 2: Set the default value of debug_mode to False

# HOW TO DISABLE CHECK FRAME MODE
# 1: In launch/wr_ls1207de_udp.launch or launch/wr_ls1207de_udp_nodelet.launch
# 2: Set the checkframe = false in xml 
