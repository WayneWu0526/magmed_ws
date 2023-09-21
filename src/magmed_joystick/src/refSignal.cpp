#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <vector>

// tracking differentiator
void TD(std::vector<float>& zk, int nf, float dThetaR)
{
    float fk1 = 0.1;
    float fk2 = 1.0;
    float fR = 10.0;
    float ferror = zk[0] - dThetaR;
    zk[0] += zk[1] / nf;
    zk[1] += (-fk1 * fR * fR * ferror - fk2 * fR * zk[1]) / nf;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "referenceSignal");

    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<std_msgs::Float64MultiArray>("/magmed_joystick/referenceSignal", 1000);

    ros::Publisher pub2 = nh.advertise<std_msgs::Float64MultiArray>("/magmed_joystick/TDSignal", 1000);

    std_msgs::Float64MultiArray msg;

    std_msgs::Float64MultiArray msg2;

    int nfreq = 100;

    ros::Rate loop_rate(nfreq);

    // input reference theta
    double dThetaR[2] = {0.0, 0.0};

    // inital TD
    std::vector<float> zk = {0.0, 0.0};

    float fphaseshift = 0.0;
    float ffrequency = 2.0 * M_PI / 10.0;
    float fA = 0.25 * M_PI;

    float fstepK = 0.0;

    // wait 10s
    ros::Duration(10.0).sleep();

    // record the initial time
    ros::Time t0 = ros::Time::now();

    while(ros::ok())
    {
        // calculate the time difference to reach t0 and return seconds
        double dt = (ros::Time::now() - t0).toSec();

        // step signal
        fstepK = M_PI / 3.0;
        // if(dt < 5.0)
        // {
        //     fstepK = M_PI / 8.0;
        // }
        // else if (5.0 <= dt && dt <= 10.0)
        // {
        //     fstepK = M_PI / 4.0;
        // }
        // else if (10.0 <= dt && dt <= 20.0)
        // {
        //     fstepK = - M_PI / 4.0;
        // }
        // else if (20.0 <= dt && dt <= 25.0)
        // {
        //     fstepK = - M_PI / 8.0;
        // }
        // else
        // {
        //     fstepK = 0.0;
        // }
        // input reference theta in step wave
        dThetaR[1] = (fstepK - dThetaR[0]) / nfreq;
        dThetaR[0] = fstepK;

        // // input reference theta in sine wave
        // dThetaR[0] = fA * sin(ffrequency * dt + fphaseshift);
        // dThetaR[1] = fA * ffrequency * cos(ffrequency * dt + fphaseshift);

        // update TDSignal
        TD(zk, nfreq, dThetaR[0]);

        msg.data.clear();
        msg.data.push_back(dThetaR[0]);
        msg.data.push_back(dThetaR[1]);
        pub.publish(msg);

        msg2.data.clear();
        msg2.data.push_back(zk[0]);
        msg2.data.push_back(zk[1]);
        pub2.publish(msg2);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

