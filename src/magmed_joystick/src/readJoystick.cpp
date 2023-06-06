#include <serial/serial.h>
#include <iostream>
#include <ros/ros.h>
#define BUFFER_SIZE 31
#define TIMEOUT 10 // ms, wait for 10ms to read data from the serial port
#define BAUDRATE 115200

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "readJoystick");

    ros::NodeHandle nh;

    serial::Serial joystick;
    serial::Timeout timeout = serial::Timeout::simpleTimeout(TIMEOUT);
    joystick.setPort("/dev/pathfinder_joystick");
    joystick.setBaudrate(BAUDRATE);
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

    // check if the port is open
    if (joystick.isOpen())
    {
        ROS_INFO_STREAM("Serial Port initialized.");
    }
    else
    {
        return -1;
    }

    ros::Rate loop_rate(100);
    while(ros::ok())
    {
        // read data from the serial port
        size_t bytes_read = joystick.available();
        if(bytes_read)
        {
            uint8_t buffer[1024]; // buffer to store the data
            size_t bytes_read = joystick.read(buffer, BUFFER_SIZE);
            for(int i=0; i<bytes_read; i++)
            {
                //16进制的方式打印到屏幕
                std::cout << std::hex << (buffer[i] & 0xff) << " ";
            }
            std::cout << std::endl;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
