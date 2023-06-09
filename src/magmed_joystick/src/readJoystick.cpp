#include <serial/serial.h>
#include <iostream>
#include <ros/ros.h>
#include <vector>
#include <geometry_msgs/Twist.h>
#define BUFFER_SIZE 31 
#define TIMEOUT 10 // ms, wait for 10ms to read data from the serial port
#define BAUDRATE 115200

#define JOY1_MAX (float)500.0

// 定义手柄数据包的结构
class JoystickDataPacket
{
public:
    uint8_t startCharacter = 0xAA; // 开始字符，固定为0xAA
    uint8_t functionCode = 0x2F; // 功能码，固定为0x01
    uint8_t dataLength = 0x17; // 数据长度，固定为0x1C
    int dataStartIndex = 5; // 数据开始的索引，固定为0x05
    uint8_t endCharacter = 0x0D; // 结束字符，固定为0x0D
};

// 定义手柄
struct Joystick
{
    signed short int nJOY1[3] = {0, 0, 0};    // 大摇杆三轴
    signed short int nJOY2[2] = {0, 0};       // 1号（左）小摇杆
    signed short int nJOY3[2] = {0, 0};       // 2号（右）小摇杆
    bool bJOYD = false;                        // 大摇杆按钮
    unsigned short int POTA = 0;               // 电位器A
    unsigned short int POTB = 0;               // 电位器B
    int BANA = 0;                              // 旋转开关A
    int BANB = 0;                              // 旋转开关B
    signed short int ENCA = 0;                 // 编码器A
    signed short int ENCB = 0;                 // 编码器B
    bool TOG[5] = {false, false, false, false, false};   // 钮子开关，共5个
    bool BUT[6] = {false, false, false, false, false, false};   // 按键开关，共6个
}joystick;

typedef void (*CallbackFunction)(const std::vector<uint8_t>&);

void dataPacketCallback(const std::vector<uint8_t> &packet)
{
    JoystickDataPacket joystickDataPacket;

    // 检查数据包长度是否满足最小要求
    if (packet.size() < BUFFER_SIZE)
    {
        ROS_WARN("Invalid data packet. Packet length is too short.");
        return;
    }

    // 检查开始字符
    if (packet[0] != joystickDataPacket.startCharacter)
    {
        ROS_WARN("Invalid data packet. Incorrect start character.");
        return;
    }

    // 检查结束字符
    if (packet[packet.size() - 1] != joystickDataPacket.endCharacter)
    {
        ROS_WARN("Invalid data packet. Incorrect end character.");
        return;
    }

    // 计算CRC校验值，使用CRC-16/XMODEM算法，初始值为0x0000，多项式为0x1021，结果异或值为0x0000，数据开始于第0个字节，长度为packet.size() - 3，输出数据反转
    uint16_t crc = 0x0000;
    for (size_t i = 0; i < packet.size() - 3; i++)
    {
        crc ^= (packet[i] << 8);
        for (size_t j = 0; j < 8; j++)
        {
            if (crc & 0x8000)
            {
                crc = (crc << 1) ^ 0x1021;
            }
            else
            {
                crc = crc << 1;
            }
        }
    }
    crc = ((crc & 0xFF00) >> 8) | ((crc & 0x00FF) << 8);

    // 提取数据长度和数据区
    uint8_t dataLength = packet[4];
    std::vector<uint8_t> data(packet.begin() + joystickDataPacket.dataStartIndex, packet.begin() + joystickDataPacket.dataStartIndex + dataLength);
    
    // 检查CRC校验值
    uint16_t receivedCrc = (packet[packet.size() - 3] << 8) | packet[packet.size() - 2];
    if (crc != receivedCrc)
    {
        ROS_WARN("Invalid data packet. CRC check failed.");
        return;
    }

    // 在这里根据功能码和数据进行相应的操作
    // 把数据转换成手柄结构体
    // 数据格式：大摇杆三轴，1号（左）小摇杆，2号（右）小摇杆，电位器A，电位器B，旋转开关A，旋转开关B，编码器A，编码器B，无，大摇杆按钮，钮子开关，按键开关
    // print data
    // 将2个字节的数据转换成有符号的短整型
    joystick.nJOY1[0] = (data[1] << 8) | data[0];
    joystick.nJOY1[1] = (data[3] << 8) | data[2];
    joystick.nJOY1[2] = (data[5] << 8) | data[4];
    // 将data第22个字节的第6位转换成bool值
    joystick.bJOYD = (data[21] & 0x40) >> 6;

    if (joystick.bJOYD)
    {

    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "readJoystick");

    ros::NodeHandle nh;

    serial::Serial joystick_serial;
    serial::Timeout timeout = serial::Timeout::simpleTimeout(TIMEOUT);
    joystick_serial.setPort("/dev/pathfinder_joystick");
    joystick_serial.setBaudrate(BAUDRATE);
    joystick_serial.setTimeout(timeout);

    try
    {
        joystick_serial.open();
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR_STREAM("Unable to open port.");
        return -1;
    }

    // check if the port is open
    if (joystick_serial.isOpen())
    {
        ROS_INFO_STREAM("Serial Port initialized.");
    }
    else
    {
        return -1;
    }

    // 注册回调函数
    CallbackFunction callback = dataPacketCallback;

    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/magmed_joystick/joystick_controller", 1000);

    geometry_msgs::Twist msg;
 
    ros::Rate loop_rate(100);
    int count = 0;
    while(ros::ok())
    {
        // read data from the serial port
        size_t bytes_read = joystick_serial.available();
        if(bytes_read)
        {
            uint8_t buffer[1024]; // buffer to store the data
            size_t bytes_read = joystick_serial.read(buffer, BUFFER_SIZE);
            std::vector<uint8_t> packet(buffer, buffer + bytes_read);
            ROS_INFO("Packet received. [%d]", count++);

            // 调用回调函数解析数据包
            callback(packet);
        }

        if(joystick.bJOYD) // linear
        {
            msg.linear.x = joystick.nJOY1[0] / JOY1_MAX;
            msg.linear.y = joystick.nJOY1[1] / JOY1_MAX;
            msg.linear.z = joystick.nJOY1[1] / JOY1_MAX;
            msg.angular.x = 0.0;
            msg.angular.y = 0.0;
            msg.angular.z = 0.0;
        }
        else
        {
            msg.angular.x = joystick.nJOY1[0] / JOY1_MAX;
            msg.angular.y = joystick.nJOY1[1] / JOY1_MAX;
            msg.angular.z = joystick.nJOY1[1] / JOY1_MAX;
            msg.linear.x = 0.0;
            msg.linear.y = 0.0;
            msg.linear.z = 0.0;
        }

        pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
