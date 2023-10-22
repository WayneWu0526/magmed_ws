#include "magmed_controller/velCtrlNode.h"

int main(int argc, char *argv[])
{   
    ros::init(argc, argv, "velocityController");
    ros::NodeHandle nh("~");

    VelCtrlNode velCtrlNode(nh);
    velCtrlNode.run();

    return 0;
}