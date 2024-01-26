#include "magrobo_feeder.hpp"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "servo");
    ros::NodeHandle nh("~");

    servo::Servo servo(nh);
    servo.run();

    return 0;
}