#pragma once

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/QR>
#include <modern_robotics.h>
#include <qpOASES.hpp>
#include <cmath>
#include <iostream>

using namespace Eigen;
using namespace qpOASES;
using namespace mr;

#define JOINTNUM 7
#define TCPNUM 6
#define INPUTNUM 5

struct MagPose {
    double psi;
    Eigen::Vector3d pos;
};