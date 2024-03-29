#pragma once

#include <serial/serial.h>
#include "std_msgs/UInt32.h"
#include "ros/ros.h"
#include <vector>

namespace servo
{

    const int BUFFER_SIZE = 8;
    const int TIMEOUT = 10; // ms, wait for 10ms to read data from the serial port
    const int BAUDRATE = 38400;
    unsigned char addr = 0x01;

    class velCommand
    {
    public:
        uint32_t VEL = 0x00000000;
        void feed(std_msgs::UInt32ConstPtr pMsg);
        velCommand(){};
    };

    class Servo
    {
    public:
        Servo(){};

        // ~Servo(){};

        Servo(ros::NodeHandle &nh) : nh(nh)
        {
            vel_sub = nh.subscribe<std_msgs::UInt32>("/magmed_feeder/vel",
                                                     10,
                                                     boost::bind(&velCommand::feed, &velCmd, _1));
        }

        ~Servo(){};

        int openSerialPort();

        int closeSerialPort();

        int activateVelMode();

        int destroyVelMode();

        int setVel(uint32_t vel);

        void run();

    private:
        int nRet = 0;
        ros::NodeHandle nh;
        ros::Subscriber vel_sub;
        serial::Serial feeder_serial;
        velCommand velCmd;
        // write single
        int writeModbus(unsigned char addr, uint16_t reg, uint16_t value);
        // write multiple
        int writeModbus_ex(unsigned char addr, uint16_t start_reg, uint16_t num_reg, const std::vector<uint16_t> &values);
        // read modbus
        int readModbus(unsigned char addr, uint16_t reg, uint16_t regNum);
        // crc_modebus check
        uint16_t crc16_modbus(uint8_t *data, uint16_t len);
        // read serialPort
        int readSerialPort(std::vector<uint8_t> &packet, bool PRINTDATAFLAG);
    };

    void Servo::run()
    {
        ros::Rate loop_rete(100);

        // 1. Open the serial port
        nRet = openSerialPort();
        if (nRet < 0)
        {
            printf("Opening serial port failed.\n");
            return;
        }
        else
        {
            printf("Opening serial port successfully.\n");
        }
        ros::Duration(1.0).sleep(); // sleep for 1 second to wait the serial port to be ready
        // 2. set the servo mode to velocity mode
        nRet = activateVelMode();
        if (nRet < 0)
        {
            printf("Set velocity mode failed.\n");
            return;
        }
        else
        {
            printf("Set velocity mode successfully.\n");
        }
        while (ros::ok())
        {
            // 3. write MODBUS to the servo
            // printf("VEL: 0x%x\n", velCmd.VEL);
            nRet = setVel(velCmd.VEL);
            // nRet = setVel(static_cast<uint32_t>(-20000)); // 0x4e20-> 20000
            if (nRet < 0)
            {
                printf("Write data failed.\n");
                break;
            }
            ros::spinOnce();
            loop_rete.sleep();
        }

        // 4. destroy the velocity mode
        nRet = destroyVelMode();
        if (nRet < 0)
        {
            printf("Destroy velocity mode failed.\n");
            return;
        }
        else
        {
            printf("Destroy velocity mode successfully.\n");
        }

        // 6. Close the serial port
        nRet = closeSerialPort();
        if (nRet < 0)
        {
            printf("Closing serial port failed.\n");
            return;
        }
        ros::shutdown();
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
        // printf("set: ");
        // for (int i = 0; i < 8; ++i)
        // {
        //     printf("0x%x ", data[i]);
        // }
        // printf("\n");
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

        usleep(1000 * TIMEOUT);
        std::vector<uint8_t> packet;
        nRet = readSerialPort(packet, false);
        if (nRet < 0)
        {
            ROS_ERROR_STREAM("Read data failed.\n");
            return -1;
        }
        if (data[4] == packet[4] && data[5] == packet[5])
        {
            // printf("Write data successfully.\n");
        }
        else
        {
            ROS_ERROR_STREAM("Write data failed.\n");
            return -1;
        }
        return 0;
    }

    int Servo::writeModbus_ex(unsigned char addr, uint16_t start_reg, uint16_t num_reg, const std::vector<uint16_t> &values)
    {
        // 检查 values 是否为空
        if (values.empty())
        {
            printf("Values vector is empty.\n");
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
            feeder_serial.write(data, 9 + num_bytes); // 9 是请求帧的长度
        }
        catch (serial::IOException &e)
        {
            ROS_ERROR_STREAM("Unable to write data to the serial port.");
            return -1;
        }

        usleep(1000 * TIMEOUT);
        std::vector<uint8_t> packet;
        nRet = readSerialPort(packet, false);
        if (nRet < 0)
        {
            ROS_ERROR_STREAM("Read data failed.\n");
            return -1;
        }
        // if (data[4] == packet[4] && data[5] == packet[5])
        // {
        //     // printf("Write data successfully.\n");
        // }
        // else
        // {
        //     ROS_ERROR_STREAM("Write data failed.\n");
        //     return -1;
        // }
        return 0;

        return 0; // 成功发送请求
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
        // feeder_serial.flush();
        try
        {
            feeder_serial.write(data, 8);
        }
        catch (serial::IOException &e)
        {
            ROS_ERROR_STREAM("Unable to write data to the serial port.");
            return -1;
        }

        usleep(1000 * TIMEOUT);
        std::vector<uint8_t> packet;
        nRet = readSerialPort(packet, true);
        if (nRet < 0)
        {
            ROS_ERROR_STREAM("Read data failed.\n");
            return -1;
        }
        return 0;
    }

    int Servo::activateVelMode()
    {
        
        nRet = writeModbus(addr, 0x003c, 0x0002);
        if (nRet < 0)
        {
            printf("Write velocity failed.\n");
            return -1;
        }
        // 设置松轴
        nRet = writeModbus(addr, 0x003e, 0x0006); 
        if (nRet < 0)
        {
            printf("Failure to release the shaft\n");
            return -1;
        }

        // 设置上电
        nRet = writeModbus(addr, 0x00b2, 0x0001);
        if (nRet < 0)
        {
            printf("Failure to power on.\n");
            return -1;
        }
        return 0;
    }

    int Servo::destroyVelMode()
    {
        // 4. 设置速度为0
        uint16_t start_reg = 0x0042;                     // 设置起始寄存器地址
        uint16_t num_reg = 0x0002;                       // 设置寄存器数量
        std::vector<uint16_t> values = {0x0000, 0x0000}; // 两个寄存器的数据值
        nRet = writeModbus_ex(addr, start_reg, num_reg, values);
        if (nRet < 0)
        {
            printf("Write data failed.\n");
            return -1;
        }
        // 5. 设置下电
        nRet = writeModbus(addr, 0x00b2, 0x0000);
        if (nRet < 0)
        {
            printf("Failure to power off.\n");
            return -1;
        }
        return 0;
    }

    int Servo::setVel(uint32_t vel)
    {
        uint16_t start_reg = 0x0042;                     // 设置起始寄存器地址
        uint16_t num_reg = 0x0002;                       // 设置寄存器数量
        uint16_t value_high = (vel >> 16) & 0xFFFF;  // 速度高位
        uint16_t value_low = vel & 0xFFFF;           // 速度低位
        std::vector<uint16_t> values = {value_high, value_low}; // 两个寄存器的数据值
        // std::vector<uint16_t> values = {0x0000, 0x0000}; // 两个寄存器的数据值
        nRet = writeModbus_ex(addr, start_reg, num_reg, values);
        if (nRet < 0)
        {
            printf("Set velocity failed.\n");
            return -1;
        }
        else
        {
            // printf("Set velocity successfully.\n");
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

    int Servo::readSerialPort(std::vector<uint8_t> &packet, bool PRINTDATAFLAG)
    {
        size_t bytes_available = feeder_serial.available();
        if (bytes_available == 0)
        {
            printf("No data available.\n");
            return -1;
        }
        uint8_t response[1024] = {0};
        try
        {
            size_t bytes_read = feeder_serial.read(response, sizeof(response));
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
            ROS_ERROR_STREAM("Unable to write data to the serial port.");
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

    void velCommand::feed(std_msgs::UInt32ConstPtr pMsg)
    {
        VEL = pMsg->data;
    };

} // namespace servo
