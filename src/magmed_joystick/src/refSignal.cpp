#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "referenceSignal");

    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<std_msgs::Float64MultiArray>("/magmed_joystick/referenceSignal", 1000);

    std_msgs::Float64MultiArray msg;

    ros::Rate loop_rate(100);

    // input reference theta
    double dThetaR[2] = {0.0, 0.0};

    // record the initial time
    ros::Time t0 = ros::Time::now();

    float fphaseshift = 0.0;
    float ffrequency = 2.0 * M_PI / 10.0;
    float fA = 0.25 * M_PI;

    while(ros::ok())
    {
        // calculate the time difference to reach t0 and return seconds
        double dt = (ros::Time::now() - t0).toSec();

        // input reference theta in sine wave
        dThetaR[0] = fA * sin(ffrequency * dt + fphaseshift);
        dThetaR[1] = fA * ffrequency * cos(ffrequency * dt + fphaseshift);

        msg.data.clear();
        msg.data.push_back(dThetaR[0]);
        msg.data.push_back(dThetaR[1]);
        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

