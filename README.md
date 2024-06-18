# magmed_ws
ROS workspace for magmedical robotics projects

Including camera, robot arm, 3DConnexion joystick, and magnetic soft continuum robots.


# To run

catkin_make --pkg magmed_msgs
ctrl+b

# To run with debug
catkin_make -DCMAKE_BUILD_TYPE=Debug

# To run in specific package
catkin_make -DCATKIN_WHITELIST_PACKAGES="magmed_msgs"
catkin_make -DCATKIN_WHITELIST_PACKAGES=" "

Zhang@Zlab:~$ ./magmed_ws/manuallyCtrl.sh
# To stop
ctrl+c

# To telerobotic run:
sudo vim /etc/hosts
(add master ip and host ip)

sudo vim ~/.bashrc
(reset the ROS_IP and ROS_MASTER_URI)

(do it on both master and host)