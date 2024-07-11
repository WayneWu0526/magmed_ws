#include "magmed_joystick/PathFinder.h"

int JoystickReader::run()
{
    uint8_t buffer[1024] = {0x00}; // buffer to store the data
    // read data from the serial port
    size_t bytes_read = joystick_serial.available();
    if (bytes_read > 0)
    {
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
        ROS_WARN("[magmed_joystick] Invalid data packet. Packet length is too short.");
        return;
    }

    // 检查开始字符
    if (packet[0] != joystickDataPacket.startCharacter)
    {
        ROS_WARN("[magmed_joystick] Invalid data packet. Incorrect start character.");
        return;
    }

    // 检查结束字符
    if (packet[packet.size() - 1] != joystickDataPacket.endCharacter)
    {
        ROS_WARN("[magmed_joystick] Invalid data packet. Incorrect end character.");
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
        ROS_WARN("[magmed_joystick] Invalid data packet. CRC check failed.");
        return;
    }

    // 在这里根据功能码和数据进行相应的操作
    // 把数据转换成手柄结构体
    // 数据格式：大摇杆三轴，1号（左）小摇杆，2号（右）小摇杆，电位器A，电位器B，旋转开关A，旋转开关B，编码器A，编码器B，无，大摇杆按钮，钮子开关，按键开关
    // print data
    // 大摇杆三轴
    joystick.nJOY1[0] = (data[1] << 8) | data[0];
    joystick.nJOY1[1] = (data[3] << 8) | data[2];
    joystick.nJOY1[2] = (data[5] << 8) | data[4];
    // set deadzone
    deadzone(joystick.nJOY1[0], JOY1_DEADZONE);
    deadzone(joystick.nJOY1[1], JOY1_DEADZONE);
    deadzone(joystick.nJOY1[2], JOY1_DEADZONE);
    // 1号（左）小摇杆
    joystick.nJOY2[0] = (data[7] << 8) | data[6];
    joystick.nJOY2[1] = (data[9] << 8) | data[8];
    deadzone(joystick.nJOY2[0], JOY2_DEADZONE);
    deadzone(joystick.nJOY2[1], JOY2_DEADZONE);
    // 2号（右）小摇杆
    joystick.nJOY3[0] = (data[11] << 8) | data[10];
    joystick.nJOY3[1] = (data[13] << 8) | data[12];
    deadzone(joystick.nJOY3[0], JOY2_DEADZONE);
    deadzone(joystick.nJOY3[1], JOY2_DEADZONE);
    // 电位器A, 电位器B
    joystick.POTA = (data[15] << 8) | data[14];
    joystick.POTB = (data[17] << 8) | data[16];
    // 旋转开关A、B
    // 获取高四位并转换为1到8的整数
    joystick.BANB = (data[18] >> 4) & 0x0F;
    // 获取低四位并转换为1到8的整数
    joystick.BANA = (data[18]) & 0x0F; // 转换为1到8的整数
    // 编码器A
    joystick.ENCA = static_cast<int8_t>(data[19]);
    joystick.ENCB = static_cast<int8_t>(data[20]);
    //大摇杆按钮
    joystick.bJOYD = (data[21] & 0x40) >> 6;
    // 扭子开关
    for (int i = 0; i < 5; ++i) {
        joystick.TOG[i] = (data[21] & (1 << i)) >> i;
    }
    // 按键开关
    for (int i = 0; i < 6; ++i) {
        joystick.BUT[i] = (data[22] & (1 << i)) >> i;
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
        // joystick_serial.flush();
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR_STREAM("[magmed_joystick] Unable to open port.");
        return -1;
    }

    // check if the port is open
    if (joystick_serial.isOpen())
    {
        ROS_INFO_STREAM("[magmed_joystick] Serial Port initialized.");
    }
    else
    {
        return -1;
    }
    return 0;
};

int JoystickReader::closeSerialPort()
{
    joystick_serial.flush();
    joystick_serial.close();
    if (!joystick_serial.isOpen())
    {
        ROS_INFO_STREAM("[magmed_joystick] Serial Port closed.");
    }
    else
    {
        return -1;
    }
    return 0;
};

// deadzone
void JoystickReader::deadzone(signed int short &nJOY, unsigned short int DEADZONE)
{
    if (nJOY > -DEADZONE && nJOY < DEADZONE)
    {
        nJOY = 0;
    }
    else if (nJOY >= DEADZONE)
    {
        nJOY -= DEADZONE;
    }
    else if (nJOY <= -DEADZONE)
    {
        nJOY += DEADZONE;
    }
};

void JoystickReader::flush()
{
    joystick_serial.flush();
};
