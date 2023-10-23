#include "magmed_joystick/PathFinder.h"

int JoystickReader::run()
{
    // read data from the serial port
    size_t bytes_read = joystick_serial.available();
    if (bytes_read)
    {
        uint8_t buffer[1024]; // buffer to store the data
        size_t bytes_read = joystick_serial.read(buffer, BUFFER_SIZE);
        std::vector<uint8_t> packet(buffer, buffer + bytes_read);

        // 调用回调函数解析数据包
        HandlePacket(packet);
        return 0;
    }
    else{
        return -1;
    }
};

void JoystickReader::HandlePacket(const std::vector<uint8_t> &packet)
{

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
    else
    {
        ROS_INFO("CRC check passed.");
    }

    // 在这里根据功能码和数据进行相应的操作
    // 把数据转换成手柄结构体
    // 数据格式：大摇杆三轴，1号（左）小摇杆，2号（右）小摇杆，电位器A，电位器B，旋转开关A，旋转开关B，编码器A，编码器B，无，大摇杆按钮，钮子开关，按键开关
    // print data
    // 将2个字节的数据转换成有符号的短整型
    joystick.nJOY1[0] = (data[1] << 8) | data[0];
    joystick.nJOY1[1] = (data[3] << 8) | data[2];
    joystick.nJOY1[2] = (data[5] << 8) | data[4];
    // 为短整型添加死区
    // 设置死区大小
    deadzone(joystick.nJOY1[0]);
    deadzone(joystick.nJOY1[1]);
    deadzone(joystick.nJOY1[2]);
    // 将data第22个字节的第6位转换成bool值
    joystick.bJOYD = (data[21] & 0x40) >> 6;

    if (joystick.bJOYD)
    {
    }
};

int JoystickReader::openSerialPort()
{
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
    return 0;
};

// deadzone
void JoystickReader::deadzone(signed int short &nJOY)
{
    if (nJOY > -JOY1_DEADZONE && nJOY < JOY1_DEADZONE)
    {
        nJOY = 0;
    }
    else if (nJOY >= JOY1_DEADZONE)
    {
        nJOY -= JOY1_DEADZONE;
    }
    else if (nJOY <= -JOY1_DEADZONE)
    {
        nJOY += JOY1_DEADZONE;
    }
};
