#include "magmed_controller/diffKine.h"
// #include "magmed_msgs/MagPose.h"
#include "magmed_controller/velCtrlDef.h"
#include <iostream>

#define JOINTNUM 7

enum_CTRLMODE CTRLMODE = enum_CTRLMODE::NM;

int main(int argc, char *argv[])
{
    /* code */
    // get cmd input, if yes, continue

    double thetalist[JOINTNUM] = {-0.0066, 0.6378, -0.0785, 2.6721, -0.0708, 1.7405, 0.0349};

    diffKine dk;

    // dk.initConfig(thetalist);

    // 接收来自控制台的输入以继续

    float h = 1e-2;
    float tolSimTime = 10.0;
    float t = 0.0;
    double refPhi[2] = {0.0};
    // refPhi.phi = 0.0;
    refPhi[1] = M_PI / (2.0 * tolSimTime);
    VectorXd JointsVels(JOINTNUM);
    while (t < tolSimTime)
    {
        dk.magTwist.psi = 0.03;
        dk.magTwist.pos = {0.0, 0.03, 0.0};
        // JointsVels = dk.jacobiMap(refPhi, thetalist, CTRLMODE);
        
        // refPhi.phi += refPhi.dphi * h;
        // update thetalist
        for (int i = 0; i < JOINTNUM; i++)
        {
            thetalist[i] += JointsVels(i) * h;
        }
        t += h;
    };
    std::cout << "JointsVels: " << std::endl;
    std::cout << JointsVels << std::endl;
    return 0;
}
