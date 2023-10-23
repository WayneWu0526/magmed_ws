#include "magmed_joystick/PathFinder.h"
#include <geometry_msgs/Twist.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "readJoystick");

    ros::NodeHandle nh;

    JoystickReader reader;

    serial::Timeout timeout = serial::Timeout::simpleTimeout(TIMEOUT);
    reader.joystick_serial.setPort("/dev/pathfinder_joystick");
    reader.joystick_serial.setBaudrate(BAUDRATE);
    reader.joystick_serial.setTimeout(timeout);

    if(reader.openSerialPort() < 0)
    {
        ROS_ERROR_STREAM("Unable to open port.");
        return -1;
    }
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/magmed_joystick/joystick_controller", 1000);

    geometry_msgs::Twist msg;
 
    ros::Rate loop_rate(100);
    int count = 0;
    while(ros::ok())
    {
        int ret = reader.run();
        if(ret < 0)
        {
            // ROS_WARN("No data received.");
            continue;
        }
        if(reader.joystick.bJOYD) // linear
        {

            msg.linear.x = reader.joystick.nJOY1[0] / (JOY1_MAX - JOY1_DEADZONE);
            msg.linear.y = reader.joystick.nJOY1[1] / (JOY1_MAX - JOY1_DEADZONE);
            msg.linear.z = reader.joystick.nJOY1[2] / (JOY1_MAX - JOY1_DEADZONE);
            msg.angular.x = 0.0;
            msg.angular.y = 0.0;
            msg.angular.z = 0.0;
        }
        else
        {
            msg.angular.x = reader.joystick.nJOY1[0] / (JOY1_MAX - JOY1_DEADZONE);
            msg.angular.y = reader.joystick.nJOY1[1] / (JOY1_MAX - JOY1_DEADZONE);
            msg.angular.z = reader.joystick.nJOY1[2] / (JOY1_MAX - JOY1_DEADZONE);
            msg.linear.x = 0.0;
            msg.linear.y = 0.0;
            msg.linear.z = 0.0;
        }

        pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    reader.joystick_serial.close();

    return 0;
}
