#include "magmed_joystick/PathFinder.h"
#include "magmed_msgs/JoyRef.h"
#include <eigen3/Eigen/Dense>
const int FREQ = 100;
const float JOY1_GAIN = 0.1;

Eigen::Vector2d TD(Eigen::Vector2d &x, double v)
{
    double k1 = 40.0; // 增大k1,提升响应速度
    double k2 = 80.0; // 增大k2,增大阻尼
    double R = 1.0;
    double T = 1.0 / FREQ;

    Eigen::Matrix2d A;
    A << 0.0, 1.0,
        -k1 * R * R, -k2 * R;
    Eigen::Vector2d B;
    B << 0.0, k1 * R * R;

    x = (Eigen::Matrix2d::Identity() + T * A) * x + T * B * v;
    return x;
};

// input: reader.joystick.nJOY1, output: refAngle
void calcuPolar(magmed_msgs::JoyRef &joyRef, const signed short *nJOY1)
{
    static double refAngle[2] = {0.0, 0.0};
    double vphi = atan2(nJOY1[0] / JOY1_MAX, nJOY1[1] / JOY1_MAX);
    double vtheta = M_PI / 3.0 * sqrt(pow(nJOY1[0] / JOY1_MAX, 2.0) + pow(nJOY1[1] / JOY1_MAX, 2.0));
    // joyRef.refPhi.phi = atan2(nJOY1[0] / JOY1_MAX, nJOY1[1] / JOY1_MAX);
    // joyRef.refTheta.theta = M_PI / 2.0 * sqrt(pow(nJOY1[0] / JOY1_MAX, 2.0) + pow(nJOY1[1] / JOY1_MAX, 2.0));

    // joyRef.refPhi.dphi = (joyRef.refPhi.phi - refAngle[0]) / 0.01;
    // joyRef.refTheta.dtheta = (joyRef.refTheta.theta - refAngle[1]) / 0.01;

    // refAngle[0] = joyRef.refPhi.phi;
    // refAngle[1] = joyRef.refTheta.theta;
    static Eigen::Vector2d xtheta{0.0, 0.0};
    static Eigen::Vector2d xphi{0.0, 0.0};
    Eigen::Vector2d phi_TD = TD(xphi, vphi);
    Eigen::Vector2d theta_TD = TD(xtheta, vtheta);
    /******************闭环控制，请取消下列注释*****************/
    // joyRef.refPhi.phi = phi_TD(0);
    // joyRef.refPhi.dphi = phi_TD(1);
    // joyRef.refTheta.theta = theta_TD(0);
    // joyRef.refTheta.dtheta = theta_TD(1);
    /******************开环控制，请取消下列注释*****************/
    joyRef.refTheta.theta = 0.0;
    joyRef.refTheta.dtheta = JOY1_GAIN * nJOY1[2] / JOY1_MAX;
    joyRef.refPhi.phi = 0.0;
    joyRef.refPhi.dphi = JOY1_GAIN * nJOY1[1] / JOY1_MAX;
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "polarRef");
    ros::NodeHandle nh("~");
    ros::Publisher pub = nh.advertise<magmed_msgs::JoyRef>("/magmed_joystick/joyRef", 1000);
    ros::Rate loop_rate(FREQ);
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
                ROS_INFO("Data received. Decoding.../n");
                calcuPolar(joyRef, reader.joystick.nJOY1);

                std::cout << "refAngle: " << joyRef.refPhi.phi << ", " << joyRef.refTheta.theta << std::endl;
                std::cout << "refdAngle: " << joyRef.refPhi.dphi << ", " << joyRef.refTheta.dtheta << std::endl;

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