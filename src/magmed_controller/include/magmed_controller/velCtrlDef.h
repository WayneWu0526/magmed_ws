#pragma once

// #include <ceres/ceres.h>
#include <eigen3/Eigen/Dense>
#include "nlopt.hpp"
// #include <eigen3/Eigen/Geometry>
#include <fstream>
#include <sstream>
#include <modern_robotics.h>
#include <qpOASES.hpp>
#include <cmath>
#include <algorithm>
#include <utility>
#include <tuple>
#include <iostream>
#include <yaml-cpp/yaml.h>
#include "magmed_msgs/PFjoystick.h"
#include "magmed_msgs/RoboStates.h"
#include "magmed_msgs/RoboJoints.h"
#include "magmed_msgs/TipAngle.h"
#include "magmed_msgs/PoseTwist.h"
#include "magmed_msgs/MagCR.h"

#include "magmed_msgs/SelfCollisionCheck.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/UInt32MultiArray.h>
#include <std_msgs/Float64.h>

using namespace Eigen;
using namespace qpOASES;
using namespace mr;

const int JOINTNUM = 7;
const int TCPNUM = 6;
const int INPUTNUM = 5;
const int CTRLFREQ = 100;

struct MagPose
{
    double psi;
    Eigen::Vector3d pos;
    MagPose(){};
};

struct Feeder
{
    uint32_t feeder_vel_FB = 0x00000000;
    uint32_t feeder_vel_UD = 0x00000000;
    int FEEDER_VEL_GAIN_FB = 0;
    int FEEDER_VEL_GAIN_UD = 0;
    // pub feeder
    std_msgs::UInt32MultiArray feeder_msg;
    // Feeder(int FEEDER_VEL_GAIN) : FEEDER_VEL_GAIN(FEEDER_VEL_GAIN) {};
    Feeder(){
        feeder_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
        feeder_msg.layout.dim[0].label = "feeder_vel";
        feeder_msg.layout.dim[0].size = 2;
        feeder_msg.layout.dim[0].stride = 2;
    };
};

struct DianaTcp
{
    float TCP_VEL_GAIN = 0.0;
    double tcp_vel[TCPNUM] = {0.0};
    double tcp_pos[TCPNUM] = {0.0};
    double tcp_acc[TCPNUM] = {0.0};

    DianaTcp(){};
};

// magnetic continuum robot
struct MagCR
{

    double phi[2] = {0.0, 0.0};
    double theta[2] = {0.0, 0.0};

    MagCR(){};
};

// define control mode
enum enum_CTRLMODE
{
    NM = 0,
    SM = 1,
    DM = 2,
    TRANS = 3
};

// define transition method
enum enum_TRANSMETHOD
{
    NT = 0,
    OCT = 1,
    OFT = 2,
    optOFT = 3
};