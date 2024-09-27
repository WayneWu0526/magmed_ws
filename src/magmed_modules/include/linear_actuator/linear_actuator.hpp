#pragma once

#include "libusb-1.0/libusb.h"
#include "linear_actuator/ch9326.h"
#include <iostream>
#include <ros/ros.h>
#include <pthread.h>
#include <std_msgs/Float64MultiArray.h>
// #include "geometry_msgs/TwistStamped.h"
#include <std_msgs/Float64.h>
#include <atomic>

namespace linear_actuator
{
    int GAIN = 0;
    const int FREQUENCY = 3200; // Hz

    // joystick input (manual control)

    class Velctrl
    {
    public:
        std_msgs::Float64MultiArray vel_ctrl;
        std::atomic<bool> dir;
        std::atomic<int> speed;
        double input_vel;
        Velctrl(): dir(0), speed(0), input_vel(0) {
            vel_ctrl.data.resize(2);
            vel_ctrl.data[0] = 0.0;
            vel_ctrl.data[1] = 0.0;
        };
        void feed(std_msgs::Float64MultiArrayConstPtr pMsg)
        {
            vel_ctrl = *pMsg;
            GAIN = (int) (vel_ctrl.data[0] / 1000.0 * 10.0);
            // printf("GAIN: %d\n", GAIN);
            // std::cout << "GAIN:" << vel_ctrl.data[0] << std::endl;
            dir = vel_ctrl.data[1] > 0 ? 1 : 0;
            speed = (int) (std::fabs(vel_ctrl.data[1]) * GAIN * FREQUENCY);
            // std::cout << "speed:" << vel_ctrl.data[1] << std::endl;
        }
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
        ros::Subscriber vel_ctrl_sub;
        ros::Publisher linear_actuator_vel_pub;
        Velctrl velctrl;
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
