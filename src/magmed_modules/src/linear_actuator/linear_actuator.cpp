#include "linear_actuator/linear_actuator.hpp"

namespace linear_actuator
{
    // 在命名空间内定义全局指针
    LinearActuator* g_actuator_ptr = nullptr;

    LinearActuator::LinearActuator(ros::NodeHandle &nh) : nh(nh), running_(true)
    {
        joystick_sub = nh.subscribe<magmed_msgs::PFjoystick>("/magmed_joystick/joystick_controller",
                                                             10,
                                                             boost::bind(&Joystick::feed, &joystick, _1));
        linear_actuator_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("linear_actuator/gframe_twist", 10);
    }

    LinearActuator::~LinearActuator()
    {
        stop();
    }

    void LinearActuator::run()
    {
        ros::Rate loop_rate(100);

        unsigned char device_count = ch9326_find();
        if (device_count == 0) {
            ROS_ERROR("[magmed_modules_linear_actuator] CH9326 device not found\n");
            return;
        }

        device_handle = ch9326_open(0);
        if (device_handle == NULL) {
            printf("[magmed_modules_linear_actuator] Failed to open CH9326 device\n");
            return;
        }

        if (ch9326_set_gpiodir(0, gpio_dir) != 1) {
            printf("[magmed_modules_linear_actuator] Failed to set GPIO input/output check root\n");
            ch9326_close(0);
            return;
        }

        pthread_create(&pwm_thread_, NULL, generatePWM, this);

        ros::Duration(1.0).sleep();
        while (ros::ok()) {
            double VEL_FACTOR = 0.01 / 7.76 / 1.04 / 5.00;
            geometry_msgs::TwistStamped linear_actuator_vel;
            linear_actuator_vel.header.stamp = ros::Time::now();
            linear_actuator_vel.twist.linear.x = VEL_FACTOR * joystick.pf_joystick.nJOY3[0] * GAIN;
            // std::cout << joystick.pf_joystick.nJOY3[0] << std::endl;
            // std::cout << linear_actuator_vel.twist.linear.x << std::endl;
            linear_actuator_vel.twist.linear.y = 0.0;
            linear_actuator_vel.twist.linear.z = 0.0;
            linear_actuator_vel.twist.angular.x = 0.0;
            linear_actuator_vel.twist.angular.y = 0.0;
            linear_actuator_vel.twist.angular.z = 0.0;
            linear_actuator_vel_pub.publish(linear_actuator_vel);    

            ros::spinOnce();
            loop_rate.sleep();
        }

        stop();
    }

    void LinearActuator::stop()
    {
        if (running_) {
            running_ = false;
            pthread_join(pwm_thread_, NULL);
            ch9326_close(0);
            ROS_INFO("[magmed_modules_linear_actuator] CH9326 closed successfully\n");
        }
    }

    void* LinearActuator::generatePWM(void* arg)
    {
        LinearActuator* actuator = (LinearActuator*)arg;
        while (actuator->running_) {
            int frequency = actuator->joystick.speed.load();
            if (frequency > 0) {
                int period = 1000000 / frequency; // 微秒
                bool direction = actuator->joystick.dir.load();
                actuator->setGPIOState(direction, true);  // 设置IO1和IO2高电平
                usleep(period / 2); // 半个周期
                actuator->setGPIOState(direction, false); // 设置IO2低电平
                usleep(period / 2); // 半个周期
            } else {
                usleep(100000); // 没有速度信号时，休眠100毫秒
            }
        }
        return NULL;
    }

    void LinearActuator::setGPIOState(bool direction, bool pulse)
    {
        if (direction) {
            gpio_data |= 0x01;
        } else {
            gpio_data &= ~0x01;
        }
        if (pulse) {
            gpio_data |= 0x02;
        } else {
            gpio_data &= ~0x02;
        }
        if (ch9326_set_gpiodata(0, gpio_data) != 1) {
            printf("[magmed_modules_linear_actuator] Failed to set GPIO\n");
            ch9326_close(0);
        }
    }
}

void signalHandler(int signum)
{
    if (linear_actuator::g_actuator_ptr) {
        linear_actuator::g_actuator_ptr->stop();
    }
    ros::shutdown();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "linear_actuator");
    ros::NodeHandle nh;

    linear_actuator::LinearActuator actuator(nh);
    linear_actuator::g_actuator_ptr = &actuator;

    signal(SIGINT, signalHandler);

    actuator.run();

    return 0;
}
