#include "magmed_controller/joyCtrlFeed.hpp"

int main(int argc, char *argv[])
{   
    ros::init(argc, argv, "joyCtrlFeed");
    ros::NodeHandle nh("~");

    joyCtrlFeed joyctrlfeed(nh);
    joyctrlfeed.run();

    return 0;
}