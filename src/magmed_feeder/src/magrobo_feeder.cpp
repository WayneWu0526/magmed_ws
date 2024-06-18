#include "magrobo_feeder.hpp"

using namespace servo;

void Servo::run(serial::Serial& serial_port, const std::string& port, const int INDX)
{
    ros::Rate loop_rete(100);

    // 1. Open the serial port
    nRet = openSerialPort(serial_port, port);
    if (nRet < 0)
    {
        // open port1 failed
        ROS_ERROR("[magmed_feeder] Opening serial port %s failed.\n", port.c_str());
        return;
    }
    else
    {
        printf("[magmed_feeder] Opening serial port %s successfully.\n", port.c_str());
    }  
    ros::Duration(1.0).sleep(); // sleep for 1 second to wait the serial port to be ready
    // 2. set the servo mode to velocity mode
    nRet = activateVelMode(serial_port);
    if (nRet < 0)
    {
        ROS_ERROR("[magmed_feeder %s] Set velocity mode failed.\n", port.c_str());
        return;
    }
    else
    {
        printf("[magmed_feeder %s] Set velocity mode successfully.\n", port.c_str());
    }
    while (ros::ok())
    {
        // 3. write MODBUS to the servo
        // printf("VEL: 0x%x\n", vel);
        nRet = setVel(serial_port, velCmd.VEL[INDX]);
        // nRet = setVel(static_cast<uint32_t>(-20000)); // 0x4e20-> 20000
        if (nRet < 0)
        {
            printf("[magmed_feeder %s] Write data failed.\n", port.c_str());
            break;
        }
        ros::spinOnce();
        loop_rete.sleep();
    }

    // 4. destroy the velocity mode
    nRet = destroyVelMode(serial_port);
    if (nRet < 0)
    {
        printf("[magmed_feeder %s] Destroy velocity mode failed.\n", port.c_str());
        return;
    }
    else
    {
        printf("[magmed_feeder %s] Destroy velocity mode successfully.\n", port.c_str());
    }

    // 6. Close the serial port
    nRet = closeSerialPort(serial_port);
    if (nRet < 0)
    {
        printf("[magmed_feeder %s] Closing serial port failed.\n", port.c_str());
        return;
    }
    ros::shutdown();
};

int Servo::writeModbus(serial::Serial& serial_port, unsigned char addr, uint16_t reg, uint16_t value)
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
    // printf("set: ");
    // for (int i = 0; i < 8; ++i)
    // {
    //     printf("0x%x ", data[i]);
    // }
    // printf("\n");
    // try-catch to write data to the serial port
    try
    {
        serial_port.write(data, 8);
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR("[magmed_feeder] Unable to write data to the serial port %s.", serial_port.getPort().c_str());
        return -1;
    }

    usleep(1000 * TIMEOUT);
    std::vector<uint8_t> packet;
    nRet = readSerialPort(serial_port, packet, false);
    if (nRet < 0)
    {
        ROS_ERROR("[magmed_feeder %s] Read data failed.\n", serial_port.getPort().c_str());
        return -1;
    }
    if (data[4] == packet[4] && data[5] == packet[5])
    {
        // printf("Write data successfully.\n");
    }
    else
    {
        ROS_ERROR("[magmed_feeder %s] Write data failed.\n", serial_port.getPort().c_str());
        return -1;
    }
    return 0;
}

int Servo::writeModbus_ex(serial::Serial& serial_port, unsigned char addr, uint16_t start_reg, uint16_t num_reg, const std::vector<uint16_t> &values)
{
    // 检查 values 是否为空
    if (values.empty())
    {
        printf("[magmed_feeder %s] Values vector is empty.\n", serial_port.getPort().c_str());
        return -1;
    }

    // 计算需要发送的字节数
    size_t num_values = values.size();
    size_t num_bytes = num_values * 2; // 每个值占2个字节

    int BUFFER_EX_SIZE = 9 + num_bytes; // 9 是地址、功能码、寄存器地址、字节数和crc校验的字节数

    // // 检查字节数是否超出BUFFER_SIZE
    // if (num_bytes > BUFFER_SIZE - 7) // 7 是地址、功能码、寄存器地址和字节数的字节数
    // {
    //     printf("Data size exceeds BUFFER_SIZE.\n");
    //     return -1;
    // }

    // 构造 Modbus 请求帧
    unsigned char data[BUFFER_EX_SIZE] = {0};
    data[0] = addr;
    data[1] = 0x10;              // 功能码 0x10 表示写多个寄存器
    data[2] = start_reg >> 8;    // 起始寄存器地址高字节
    data[3] = start_reg & 0xFF;  // 起始寄存器地址低字节
    data[4] = num_reg >> 8;   // 寄存器数量高字节
    data[5] = num_reg & 0xFF; // 寄存器数量低字节
    data[6] = num_bytes;         // 字节数
    for (size_t i = 0; i < num_values; ++i)
    {
        uint16_t value = values[i];
        data[7 + i * 2] = value >> 8;   // 数据高字节
        data[8 + i * 2] = value & 0xFF; // 数据低字节
    }

    // 计算 CRC 校验
    uint16_t crc = crc16_modbus(data, BUFFER_EX_SIZE - 2);
    data[7 + num_bytes] = crc & 0xFF; // CRC 低字节
    data[8 + num_bytes] = crc >> 8;   // CRC 高字节
    // printf("set: ");
    // for (int i = 0; i < sizeof(data); ++i)
    // {
    //     printf("0x%x ", data[i]);
    // }
    // printf("\n");
    // 发送请求帧
    try
    {
        serial_port.write(data, 9 + num_bytes); // 9 是请求帧的长度
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR("[magmed_feeder] Unable to write data to the serial port %s.", serial_port.getPort().c_str());
        return -1;
    }

    usleep(1000 * TIMEOUT);
    std::vector<uint8_t> packet;
    nRet = readSerialPort(serial_port, packet, false);
    if (nRet < 0)
    {
        ROS_ERROR("[magmed_feeder %s] Read data failed.\n", serial_port.getPort().c_str());
        return -1;
    }
    // if (data[4] == packet[4] && data[5] == packet[5])
    // {
    //     // printf("Write data successfully.\n");
    // }
    // else
    // {
    //     ROS_ERROR("Write data failed.\n");
    //     return -1;
    // }

    return 0; // 成功发送请求
}

int Servo::readModbus(serial::Serial& serial_port, unsigned char addr, uint16_t reg, uint16_t regNum)
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
    // feeder_serial.flush();
    try
    {
        serial_port.write(data, 8);
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR("[magmed_feeder] Unable to write data to the serial port %s.", serial_port.getPort().c_str());
        return -1;
    }

    usleep(1000 * TIMEOUT);
    std::vector<uint8_t> packet;
    nRet = readSerialPort(serial_port, packet, true);
    if (nRet < 0)
    {
        ROS_ERROR("[magmed_feeder %s] Read data failed.\n", serial_port.getPort().c_str());
        return -1;
    }
    return 0;
}

int Servo::activateVelMode(serial::Serial& serial_port)
{
    
    nRet = writeModbus(serial_port, addr, 0x003c, 0x0002);
    if (nRet < 0)
    {
        ROS_ERROR("[magmed_feeder %s] Write velocity failed.\n", serial_port.getPort().c_str());
        return -1;
    }
    // 设置松轴
    nRet = writeModbus(serial_port, addr, 0x003e, 0x0006); 
    if (nRet < 0)
    {
        ROS_ERROR("[magmed_feeder %s] Failure to release the shaft\n", serial_port.getPort().c_str());
        return -1;
    }

    // 设置上电
    nRet = writeModbus(serial_port, addr, 0x00b2, 0x0001);
    if (nRet < 0)
    {
        printf("[magmed_feeder %s] Failure to power on.\n", serial_port.getPort().c_str());
        return -1;
    }
    return 0;
}

int Servo::destroyVelMode(serial::Serial& serial_port)
{
    // 4. 设置速度为0
    uint16_t start_reg = 0x0042;                     // 设置起始寄存器地址
    uint16_t num_reg = 0x0002;                       // 设置寄存器数量
    std::vector<uint16_t> values = {0x0000, 0x0000}; // 两个寄存器的数据值
    nRet = writeModbus_ex(serial_port, addr, start_reg, num_reg, values);
    if (nRet < 0)
    {
        printf("[magmed_feeder %s] Write data failed.\n", serial_port.getPort().c_str());
        return -1;
    }
    // 5. 设置下电
    nRet = writeModbus(serial_port, addr, 0x00b2, 0x0000);
    if (nRet < 0)
    {
        printf("[magmed_feeder %s] Failure to power off.\n", serial_port.getPort().c_str());
        return -1;
    }
    return 0;
}

int Servo::setVel(serial::Serial& serial_port, uint32_t vel)
{
    uint16_t start_reg = 0x0042;                     // 设置起始寄存器地址
    uint16_t num_reg = 0x0002;                       // 设置寄存器数量
    uint16_t value_high = (vel >> 16) & 0xFFFF;  // 速度高位
    uint16_t value_low = vel & 0xFFFF;           // 速度低位
    std::vector<uint16_t> values = {value_high, value_low}; // 两个寄存器的数据值
    // std::vector<uint16_t> values = {0x0000, 0x0000}; // 两个寄存器的数据值
    nRet = writeModbus_ex(serial_port, addr, start_reg, num_reg, values);
    if (nRet < 0)
    {
        printf("[magmed_feeder %s] Set velocity failed.\n", serial_port.getPort().c_str());
        return -1;
    }
    else
    {
        // printf("Set velocity successfully.\n");
    }
    return 0;
}

int Servo::openSerialPort(serial::Serial& serial_port, const std::string& port)
{
    serial::Timeout timeout = serial::Timeout::simpleTimeout(TIMEOUT);
    serial_port.setPort(port);
    serial_port.setBaudrate(BAUDRATE);
    serial_port.setTimeout(timeout);

    try
    {
        serial_port.open();
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR("[magmed_feeder] Unable to open port %s.", port.c_str());
        return -1;
    }

    // check if the port is open
    if (serial_port.isOpen())
    {
        ROS_INFO("[magmed_feeder] Serial Port %s initialized.", port.c_str());
    }
    else
    {
        return -1;
    }
    return 0;
}

int Servo::closeSerialPort(serial::Serial& serial_port)
{
    if (serial_port.isOpen())
    {
        serial_port.close();
        printf("[magmed_feeder] Serial Port %s closed.\n", serial_port.getPort().c_str());
    }
    return 0;
}

int Servo::readSerialPort(serial::Serial& serial_port, std::vector<uint8_t> &packet, bool PRINTDATAFLAG)
{
    size_t bytes_available = serial_port.available();
    if (bytes_available == 0)
    {
        ROS_ERROR("[magmed_feeder %s] No data available.\n", serial_port.getPort().c_str());
        return -1;
    }
    uint8_t response[1024] = {0};
    try
    {
        size_t bytes_read = serial_port.read(response, sizeof(response));
        packet.assign(response, response + bytes_read);
        // 打印 packet 中的内容
        if (PRINTDATAFLAG)
        {
            printf("response: ");
            for (size_t i = 0; i < packet.size(); ++i)
            {
                printf("0x%x ", packet[i]);
            }
            printf("\n");
        }
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR("[magmed_feeder] Unable to write data to the serial port %s.", serial_port.getPort().c_str());
        return -1;
    }
    return 0;
};

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

void velCommand::feed(const std_msgs::UInt32MultiArray::ConstPtr& pMsg)
{
    VEL[0] = - pMsg->data[0];
    VEL[1] = pMsg->data[1];
    // for (size_t i = 0; i < pMsg->data.size(); ++i)
    // {
    //     printf("0x%x ", pMsg->data[i]);
    // }
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "servo");
    ros::NodeHandle nh("~");

    servo::Servo servo(nh);
    serial::Serial feeder_serial_fb;
    serial::Serial feeder_serial_ud;
    std::string port_fb = "/dev/feeder_fb";
    std::string port_ud = "/dev/feeder_ud";
    // 这里传递进去始终是VEL_FB=0和VEL_UD=0，应该要传递的是velCmd？
    // 创建enum，用于区分VEL_FB和VEL_UD
    std::thread thread_fb(&Servo::run, &servo, std::ref(feeder_serial_fb), std::ref(port_fb), 0);
    std::thread thread_ud(&Servo::run, &servo, std::ref(feeder_serial_ud), std::ref(port_ud), 1);

    if (thread_fb.joinable())
    {
        thread_fb.join();
    }
    if (thread_ud.joinable())
    {
        thread_ud.join();
    }
    // servo.run(feeder_serial_fb, port_fb);
    // servo.run(feeder_serial_ud, port_ud);

    return 0;
}