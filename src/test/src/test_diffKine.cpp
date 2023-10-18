#include "magmed_controller/diffKine.h"
// #include "magmed_msgs/MagPose.h"
#include "magmed_controller/velCtrlDef.h"
#include <iostream>

#define JOINTNUM 7

int main(int argc, char *argv[])
{
    /* code */
    // get cmd input, if yes, continue
    diffKineParam param = diffKineParam();

    MagPose magTwist;
    magTwist.psi = 0.0;
    magTwist.pos = {0.0, 0.0, 0.0};
    double thetalist[JOINTNUM] = {-0.0066, 0.6378, -0.0785, 2.6721, -0.0708, 1.7405, 0.0349};

    double phi[2] = {0.0};
    diffKine dk = dk;
    MagPose magPose;
    dk.getMagPose(magPose, thetalist, param);
    VectorXd JointsVels = dk.jacobiMap(phi, magPose, magTwist, thetalist, param);

    std::cout << "JointsVels: " << JointsVels << std::endl;

    return 0;
}
