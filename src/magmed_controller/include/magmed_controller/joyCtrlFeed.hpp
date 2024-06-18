#pragma Once

#include <ros/ros.h>
#include "magmed_controller/velCtrlDef.h"

// joystick input (manual control)
class Joystick
{
public:
    magmed_msgs::PFjoystick joystick;
    Feeder feeder;
    DianaTcp dianaTcp;
    MagCR magCR;
    void feed(magmed_msgs::PFjoystickConstPtr pMsg);
    Joystick(){};
};

class joyCtrlFeed
{
public:
    void run();

    joyCtrlFeed(){};
    joyCtrlFeed(ros::NodeHandle &nh) : nh(nh)
    {
        joystick_sub = nh.subscribe<magmed_msgs::PFjoystick>("/magmed_joystick/joystick_controller",
                                                             10,
                                                             boost::bind(&Joystick::feed, &joystick, _1));

        feeder_vel_pub = nh.advertise<std_msgs::UInt32MultiArray>("/magmed_feeder/vel", 10);
                                
    }

private:
    Joystick joystick;

    ros::NodeHandle nh;
    ros::Subscriber joystick_sub;
    ros::Publisher feeder_vel_pub;
};
void joyCtrlFeed::run()
{
    int CTRLFREQ = 100;
    ros::Rate loop_rate(CTRLFREQ);
    while(ros::ok())
    {
        std_msgs::UInt32MultiArray feeder_msg;

        // initialize feeder_msg
        feeder_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
        feeder_msg.layout.dim[0].label = "feeder_vel";
        feeder_msg.layout.dim[0].size = 2;
        feeder_msg.layout.dim[0].stride = 2;

        feeder_msg.data.clear();
        feeder_msg.data.push_back(joystick.feeder.feeder_vel_FB);
        feeder_msg.data.push_back(joystick.feeder.feeder_vel_UD);

        // feeder_msg.data = joystick.feeder.feeder_vel;
        feeder_vel_pub.publish(feeder_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }
};

void Joystick::feed(magmed_msgs::PFjoystickConstPtr pMsg)
{
    joystick = *pMsg;
    // feeder 数据

    feeder.FEEDER_VEL_GAIN_UD = joystick.POTB * 50;
    feeder.feeder_vel_UD = static_cast<uint32_t>(feeder.FEEDER_VEL_GAIN_UD * joystick.nJOY2[1]);

    feeder.FEEDER_VEL_GAIN_FB = joystick.POTA * 50;
    feeder.feeder_vel_FB = static_cast<uint32_t>(feeder.FEEDER_VEL_GAIN_FB * joystick.nJOY3[0]);
}