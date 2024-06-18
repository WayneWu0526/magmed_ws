#pragma once

#include <serial/serial.h>
#include <ros/ros.h>
#include <vector>
const int BUFFER_SIZE = 31;
const int TIMEOUT = 10; // ms, wait for 10ms to read data from the serial port
const int BAUDRATE = 57600;

const float JOY1_MAX = 500.0;
const float JOY2_MAX = 300.0;
const float JOY3_MAX = 300.0;
const unsigned short int JOY1_DEADZONE = 30;
const unsigned short int JOY2_DEADZONE = 15;
const unsigned short int JOY3_DEADZONE = 15;

// 定义手柄数据包的结构
class JoystickDataPacket
{
public:
    uint8_t startCharacter = 0xAA; // 开始字符，固定为0xAA
    uint8_t functionCode = 0x2F;   // 功能码，固定为0x01
    uint8_t dataLength = 0x17;     // 数据长度，固定为0x1C
    int dataStartIndex = 5;        // 数据开始的索引，固定为0x05
    uint8_t endCharacter = 0x0D;   // 结束字符，固定为0x0D
    JoystickDataPacket(){};
};

// 定义手柄
struct Joystick
{
    signed short int nJOY1[3] = {0, 0, 0};                    // 大摇杆三轴
    signed short int nJOY2[2] = {0, 0};                       // 1号（左）小摇杆
    signed short int nJOY3[2] = {0, 0};                       // 2号（右）小摇杆
    bool bJOYD = false;                                       // 大摇杆按钮
    unsigned short int POTA = 0;                              // 电位器A
    unsigned short int POTB = 0;                              // 电位器B
    int BANA = 0;                                             // 旋转开关A
    int BANB = 0;                                             // 旋转开关B
    signed short int ENCA = 0;                                // 编码器A
    signed short int ENCB = 0;                                // 编码器B
    bool TOG[5] = {false, false, false, false, false};        // 钮子开关，共5个
    bool BUT[6] = {false, false, false, false, false, false}; // 按键开关，共6个

    Joystick(){};
};

class JoystickReader
{
public:
    Joystick joystick;
    serial::Serial joystick_serial;
    void HandlePacket(const std::vector<uint8_t>& packet);
    int run();
    void flush();
    int findStartPort();
    int openSerialPort();
    int closeSerialPort();
    JoystickReader() {};

private:
    JoystickDataPacket joystickDataPacket;
    void deadzone(signed int short &nJOY, unsigned short int DEADZONE);
};