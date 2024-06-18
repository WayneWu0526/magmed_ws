#include "magmed_joystick/PathFinder.h"
#include "magmed_msgs/PFjoystick.h"

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
        ROS_ERROR_STREAM("[magmed_joystick] Unable to open port.");
        return -1;
    }
    ros::Publisher pub = nh.advertise<magmed_msgs::PFjoystick>("/magmed_joystick/joystick_controller", 1000);

    magmed_msgs::PFjoystick msg;
    ros::Rate loop_rate(100);
    while(ros::ok())
    {
        int ret = reader.run();
        if(ret < 0)
        {
            // ROS_WARN("No data received.");
            continue;
        }
        msg.nJOY1[0] = reader.joystick.nJOY1[0] / JOY1_MAX;
        msg.nJOY1[1] = reader.joystick.nJOY1[1] / JOY1_MAX;
        msg.nJOY1[2] = reader.joystick.nJOY1[2] / JOY1_MAX;

        msg.nJOY2[0] = reader.joystick.nJOY2[0] / JOY2_MAX;
        msg.nJOY2[1] = reader.joystick.nJOY2[1] / JOY2_MAX;

        msg.nJOY3[0] = reader.joystick.nJOY3[0] / JOY3_MAX;
        msg.nJOY3[1] = reader.joystick.nJOY3[1] / JOY3_MAX;

        msg.bJOYD = reader.joystick.bJOYD;

        msg.BANA = reader.joystick.BANA;
        msg.BANB = reader.joystick.BANB;

        msg.POTA = reader.joystick.POTA;
        msg.POTB = reader.joystick.POTB;

        msg.ENCA = reader.joystick.ENCA;
        msg.ENCB = reader.joystick.ENCB;

        msg.TOG[0] = reader.joystick.TOG[0];
        msg.TOG[1] = reader.joystick.TOG[1];
        msg.TOG[2] = reader.joystick.TOG[2];
        msg.TOG[3] = reader.joystick.TOG[3];
        msg.TOG[4] = reader.joystick.TOG[4];

        msg.BUT[0] = reader.joystick.BUT[0];
        msg.BUT[1] = reader.joystick.BUT[1];
        msg.BUT[2] = reader.joystick.BUT[2];
        msg.BUT[3] = reader.joystick.BUT[3];
        msg.BUT[4] = reader.joystick.BUT[4];
        msg.BUT[5] = reader.joystick.BUT[5];


        pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    if(reader.closeSerialPort()<0)
    {
        ROS_ERROR_STREAM("[magmed_joystick] Unable to close port.");
        return -1;
    }

    return 0;
}
