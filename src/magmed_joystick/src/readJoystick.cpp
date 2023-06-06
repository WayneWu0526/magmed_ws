#include <serial/serial.h>
#include <iostream>
#include <ros/ros.h>

int main(int argc, char** argv[])
{
    serial::Serial joystick;
    serial::Timeout timeout = serial::Timeout::simpleTimeout(100);
    joystick.setPort("/dev/ttyACM0");
    joystick.setBaudrate(115200);
    joystick.setTimeout(timeout);

    try
    {
        joystick.open();
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR_STREAM("Unable to open port.");
        return -1;
    }
    return 0;
}
