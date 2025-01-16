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

# To disable teleoperation:
1. change 'clienthost' in '/ect/hosts' to local ip
clienthost 改为本地的.190
2. disable 'ROS_HOSTNAME' in '~/.bashrc'
注释掉ROS_HOSTNAME

To enable:

clienthost 改为主机的.35

ROS_HOSTNAME 添加从机ip

# set manually mode:
1. diana_run.h: 将STATE.VAL直接初始化为RUN

2. 将velCtrlNode.hpp中的SYSCTRLMODE设置为0

catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3

1. 修改末端关节期望为0
2. 若关节达到极限，考虑变换