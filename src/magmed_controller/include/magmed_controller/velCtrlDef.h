#pragma once

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/QR>
#include <modern_robotics.h>
#include <qpOASES.hpp>
#include <cmath>
#include <iostream>
#include "magmed_msgs/JoyRef.h"
#include "magmed_msgs/RefTheta.h"
#include "magmed_msgs/RefPhi.h"
#include "magmed_msgs/RoboStates.h"
#include "magmed_msgs/RoboJoints.h"
#include "magmed_msgs/TipAngle.h"

using namespace Eigen;
using namespace qpOASES;
using namespace mr;

#define JOINTNUM 7
#define TCPNUM 6
#define INPUTNUM 5
#define CTRLFREQ 100

struct MagPose {
    double psi;
    Eigen::Vector3d pos;
    MagPose() {};
};