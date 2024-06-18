#include "magmed_manipulator/diana_run.h"

int main(int argc, char *argv[])
{

    ros::init(argc, argv, "dianaManipulator");
    ros::NodeHandle nh("~");

    DianaStateManage manager(nh);
    manager.run();
    
    return 0;
}