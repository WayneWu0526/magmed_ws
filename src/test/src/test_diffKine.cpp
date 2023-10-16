#include "magmed_controller/diffKine.h"
#include "magmed_msgs/MagPose.h"
#include <iostream>

#define JOINTNUM 7

int main(int argc, char *argv[])
{
    /* code */
    // get cmd input, if yes, continue
    while(true)
    {
        std::cout << "Continue? (y/n)" << std::endl;
        char cmd;
        std::cin >> cmd;
        if(cmd == 'y')
        {
            break;
        }
        else if(cmd == 'n')
        {
            return 0;
        }
        else
        {
            std::cout << "Invalid input!" << std::endl;
        }
    }
    diffKineParam param = diffKineParam();

    magmed_msgs::MagPose magPos;
    magPos.psi = 0.0;
    magPos.pos = {30.0e-3, 183.0e-3, 0.0};

    magmed_msgs::MagPose magTwist;
    magTwist.psi = 0.0;
    magTwist.pos = {0.0, 0.0, 0.0};
    double thetalist[JOINTNUM] = {-0.0427, 0.7079, 0.0559, 2.0404, 0.0, 1.17677, -1.6070};

    double phi[2] = {0.0};
    diffKine dk = dk;
    VectorXd JointsVels = dk.jacobiMap(phi, magPos, magTwist, thetalist, param);
    std::cout << "JointsVels: " << JointsVels << std::endl;

    return 0;
}
