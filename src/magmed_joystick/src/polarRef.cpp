#include "magmed_joystick/PathFinder.h"
#include "magmed_msgs/JoyRef.h"

// input: reader.joystick.nJOY1, output: refAngle
void calcuPolar(magmed_msgs::JoyRef &joyRef, const signed short *nJOY1)
{   
    static double refAngle[2] = {0.0, 0.0};
    joyRef.refPhi.phi = atan2(nJOY1[0] / JOY1_MAX, nJOY1[1] / JOY1_MAX);
    joyRef.refTheta.theta = M_PI / 2.0 * sqrt(pow(nJOY1[0] / JOY1_MAX, 2.0) + pow(nJOY1[1] / JOY1_MAX, 2.0));

    joyRef.refPhi.dphi = (joyRef.refPhi.phi - refAngle[0]) / 0.01;
    joyRef.refTheta.dtheta = (joyRef.refTheta.theta - refAngle[1]) / 0.01;

    refAngle[0] = joyRef.refPhi.phi;
    refAngle[1] = joyRef.refTheta.theta;
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "polarRef");
    ros::NodeHandle nh("~");
    ros::Publisher pub = nh.advertise<magmed_msgs::JoyRef>("cmd_vel", 1000);
    ros::Rate loop_rate(100);
    magmed_msgs::JoyRef joyRef;

    JoystickReader reader;

    int ret = 0;
    do
    {
        ret = reader.openSerialPort();
        if (ret < 0)
        {
            ROS_ERROR("Open port failed.");
            break;
        }

        while (ros::ok())
        {
            ret = reader.run();
            if (ret < 0)
            {
                // ROS_WARN("No data received.");
                continue;
            }
            else
            {
                calcuPolar(joyRef, reader.joystick.nJOY1);

                std::cout << "refAngle: " << joyRef.refPhi.phi << ", " << joyRef.refTheta.theta << std::endl;
                std::cout << "refdAngle: " << joyRef.refTheta.theta << ", " << joyRef.refTheta.dtheta << std::endl;

                pub.publish(joyRef);
            }
            ros::spinOnce();
            loop_rate.sleep();
        };
    } while (0);
    // 关闭串口
    ROS_INFO("Closing serial port.");
    reader.joystick_serial.close();
    // 退出

    return 0;
};