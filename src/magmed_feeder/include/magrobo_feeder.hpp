#pragma once

#include <serial/serial.h>
#include "magmed_msgs/PFjoystick.h"
#include "ros/ros.h"
#include <vector>

namespace servo
{

const int BUFFER_SIZE = 8;
const int TIMEOUT = 10; // ms, wait for 10ms to read data from the serial port
const int BAUDRATE = 38400;

class Joystick
{
public:
    int VEL_GAIN = 3000;
    uint16_t VEL = 0x0000;
    magmed_msgs::PFjoystick joystick;
    void feed(magmed_msgs::PFjoystickConstPtr pMsg);
    Joystick() {};
};

class Servo 
{
public:

    Servo() {};

    Servo(ros::NodeHandle &nh) : nh(nh)
    {
        joystick_sub = nh.subscribe<magmed_msgs::PFjoystick>("/magmed_joystick/joystick_controller",
                                                          10,
                                                          boost::bind(&Joystick::feed, &joystick, _1));
    }

    ~Servo() {};

    int openSerialPort();

    int closeSerialPort();

    int activateVelMode();

    void run();

private:
    ros::NodeHandle nh;
    ros::Subscriber joystick_sub;
    serial::Serial feeder_serial;
    Joystick joystick;
    // crc_modebus check
    int writeModbus(unsigned char addr, uint16_t reg, uint16_t value);
    int readModbus(unsigned char addr, uint16_t reg, uint16_t regNum);
    uint16_t crc16_modbus(uint8_t *data, uint16_t len);
};

void Servo::run()
{
    int nRet;
    ros::Rate loop_rete(100);

    // 1. Open the serial port
    nRet = openSerialPort();
    if (nRet < 0)
    {
        ros::shutdown();
        return;
    }

    // 2. set the servo mode to velocity mode
    nRet = activateVelMode();
    if (nRet < 0)
    {
        ros::shutdown();
        return;
    }
    // 3. read MODBUS from the servo
    nRet = readModbus(0x01, 0x0042, 0x0001);
    if (nRet < 0)
    {
        ros::shutdown();
        return;
    }

    while(ros::ok())
    {
        // 3. write MODBUS to the servo
        // printf("VEL: 0x%x\n", joystick.VEL);
        // writeModbus(0x01, 0x0042, joystick.VEL);
        ros::spinOnce();
        loop_rete.sleep();
    }

    // 4. Close the serial port
    nRet = closeSerialPort();
    if (nRet < 0)
    {   
        printf("Closing serial port failed.\n");
        return;
    }
}

int Servo::activateVelMode()
{
    unsigned char addr = 0x01;
    uint16_t reg = 0x003c; // 速度模式寄存器地址
    uint16_t value = 0x0002; // 速度模式值
    if(writeModbus(addr, reg, value) != 0)
    {
        printf("Activate velocity mode failed.\n");
        return -1;
    }
    return 0;
}

int Servo::openSerialPort()
{
    serial::Timeout timeout = serial::Timeout::simpleTimeout(TIMEOUT);
    feeder_serial.setPort("/dev/feeder");
    feeder_serial.setBaudrate(BAUDRATE);
    feeder_serial.setTimeout(timeout);

    try
    {
        feeder_serial.open();
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR_STREAM("Unable to open port.");
        return -1;
    }

    // check if the port is open
    if (feeder_serial.isOpen())
    {
        ROS_INFO_STREAM("Serial Port initialized.");
    }
    else
    {
        return -1;
    }
    return 0;
}

int Servo::closeSerialPort()
{
    if (feeder_serial.isOpen())
    {
        feeder_serial.close();
        printf("Serial Port closed.\n");
    }
    return 0;
}

int Servo::writeModbus(unsigned char addr, uint16_t reg, uint16_t value)
{
    unsigned char data[BUFFER_SIZE] = {0};
    data[0] = addr;
    data[1] = 0x06;
    data[2] = reg >> 8;
    data[3] = reg & 0xFF;
    data[4] = value >> 8;
    data[5] = value & 0xFF;
    uint16_t crc = crc16_modbus(data, 6);
    data[6] = crc & 0xFF;
    data[7] = crc >> 8;
    // print the data
    for (int i = 0; i < 8; ++i)
    {
        printf("0x%x ", data[i]);
    }
    printf("\n");
    // try-catch to write data to the serial port
    try
    {
        feeder_serial.write(data, 8);
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR_STREAM("Unable to write data to the serial port.");
        return -1;
    }

    return 0;
}

int Servo::readModbus(unsigned char addr, uint16_t reg, uint16_t regNum)
{
    // send the request
    unsigned char data[BUFFER_SIZE] = {0};
    data[0] = addr;
    data[1] = 0x03;
    data[2] = reg >> 8;
    data[3] = reg & 0xFF;
    data[4] = regNum >> 8;
    data[5] = regNum & 0xFF;
    uint16_t crc = crc16_modbus(data, 6);
    data[6] = crc & 0xFF;
    data[7] = crc >> 8;
    // print the data
    printf("request: ");
    for (int i = 0; i < 8; ++i)
    {
        printf("0x%x ", data[i]);
    }
    printf("\n");
    // try-catch to write data to the serial port
    feeder_serial.flush();
    try
    {
        feeder_serial.write(data, 8);
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR_STREAM("Unable to write data to the serial port.");
        return -1;
    }

    usleep(500000);
    size_t bytes_available = feeder_serial.available();
    if(bytes_available == 0)
    {
        printf("No data available.\n");
        return -1;
    }
    uint8_t response[1024] = {0};
    size_t bytes_read = feeder_serial.read(response, sizeof(response));
    std::vector<uint8_t> packet(response, response + bytes_read);
    // 打印 packet 中的内容
    printf("response: ");
    for (size_t i = 0; i < packet.size(); ++i)
    {
        printf("0x%x ", packet[i]);
    }
    printf("\n");
    return 0;
}

uint16_t Servo::crc16_modbus(uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    uint16_t i, j;
    for (i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (j = 0; j < 8; j++)
        {
            if (crc & 0x0001)
            {
                crc >>= 1;
                crc ^= 0xA001;
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    return crc;
}

void Joystick::feed(magmed_msgs::PFjoystickConstPtr pMsg)
{
    // 1. Open the serial port
    joystick = *pMsg;
    // 数据高位
    VEL_GAIN = joystick.POTA * 3;
    VEL = static_cast<uint16_t>(VEL_GAIN * joystick.nJOY3[0]);
    // 数据低位
}

} // namespace servo
