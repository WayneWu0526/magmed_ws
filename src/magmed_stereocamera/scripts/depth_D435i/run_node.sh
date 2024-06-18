#!/bin/bash
# Source ROS setup file
# source /opt/ros/noetic/setup.bash
# Source the Conda environment
source ~/anaconda3/etc/profile.d/conda.sh
conda activate yolo_d435i
# Run the Python script
source /opt/ros/noetic/setup.bash
# rosrun
rosrun magmed_stereocamera rstest.py
