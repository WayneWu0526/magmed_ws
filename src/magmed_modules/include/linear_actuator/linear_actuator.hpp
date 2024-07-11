#pragma once

#include "libusb-1.0/libusb.h"
#include "linear_actuator/ch9326.h"
#include <iostream>
#include <ros/ros.h>
#include <pthread.h>
#include "magmed_msgs/PFjoystick.h"
#include "geometry_msgs/TwistStamped.h"
#include <atomic>

namespace linear_actuator
{
    int GAIN = 0;
    const int FREQUENCY = 3200; // Hz

    // joystick input (manual control)
    class Joystick
    {
    public:
        magmed_msgs::PFjoystick pf_joystick;

        std::atomic<bool> dir; // 0: forward, 1: backward
        std::atomic<int> speed; // 0: stop, 1: low speed, 2: high speed

        void feed(magmed_msgs::PFjoystickConstPtr pMsg)
        {
            pf_joystick = *pMsg;
            GAIN = (int) (pf_joystick.POTA / 1000.0 * 10.0);
            // printf("GAIN: %d\n", GAIN);
            dir = pf_joystick.nJOY3[0] > 0 ? 1 : 0;
            speed = (int) (std::fabs(pf_joystick.nJOY3[0]) * GAIN * FREQUENCY);
        }
        Joystick() : dir(0), speed(0) {};
    };

    class LinearActuator
    {
    public:
        LinearActuator(ros::NodeHandle &nh);
        ~LinearActuator();
        void run();
        void stop();

    private:
        static void* generatePWM(void* arg);
        void setGPIOState(bool direction, bool pulse);

        ros::NodeHandle nh;
        ros::Subscriber joystick_sub;
        ros::Publisher linear_actuator_vel_pub;
        Joystick joystick;
        usb_dev_handle* device_handle;
        unsigned char gpio_data = 0x00;
        unsigned char gpio_dir = 0x03; // 设置IO1和IO2为输出
        pthread_t pwm_thread_;
        bool running_;
    };

    // 在命名空间内声明全局指针
    extern LinearActuator* g_actuator_ptr;
} // namespace linear_actuator

void signalHandler(int signum);
