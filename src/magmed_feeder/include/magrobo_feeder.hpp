#pragma once

#include <serial/serial.h>
#include "std_msgs/UInt32MultiArray.h"
#include "ros/ros.h"
#include <vector>
#include <thread>

namespace servo
{

    const int BUFFER_SIZE = 8;
    const int TIMEOUT = 10; // ms, wait for 10ms to read data from the serial port
    const int BAUDRATE = 38400;
    unsigned char addr = 0x01;

    class velCommand
    {
    public:
        uint32_t VEL[2] = {0x00000000, 0x00000000}; // [0] = VEL_FB, [1] = VEL_UD
        // uint32_t VEL_FB = 0x00000000;
        // uint32_t VEL_UD = 0x00000000;
        void feed(const std_msgs::UInt32MultiArray::ConstPtr& pMsg);
        velCommand(){};
    };

    class Servo
    {
    public:

        // ~Servo(){};

        Servo(ros::NodeHandle &nh) : nh(nh)
        {
            vel_sub = nh.subscribe<std_msgs::UInt32MultiArray>("/magmed_feeder/vel",
                                                     10,
                                                     boost::bind(&velCommand::feed, &velCmd, _1));
        }

        ~Servo(){};

        int openSerialPort(serial::Serial& serial_port, const std::string& port);

        int closeSerialPort(serial::Serial& serial_port);

        int activateVelMode(serial::Serial& serial_port);

        int destroyVelMode(serial::Serial& serial_port);

        int setVel(serial::Serial& serial_port, uint32_t vel);

        void run(serial::Serial& serial_port, const std::string& port, const int INDX);

        velCommand velCmd;

        Servo(){};

    private:
        int nRet = 0;
        // std::string Port;
        ros::NodeHandle nh;
        ros::Subscriber vel_sub;
        // write single
        int writeModbus(serial::Serial& serial_port, unsigned char addr, uint16_t reg, uint16_t value);
        // write multiple
        int writeModbus_ex(serial::Serial& serial_port, unsigned char addr, uint16_t start_reg, uint16_t num_reg, const std::vector<uint16_t> &values);
        // read modbus
        int readModbus(serial::Serial& serial_port, unsigned char addr, uint16_t reg, uint16_t regNum);
        // crc_modebus check
        uint16_t crc16_modbus(uint8_t *data, uint16_t len);
        // read serialPort
        int readSerialPort(serial::Serial& serial_port, std::vector<uint8_t> &packet, bool PRINTDATAFLAG);
    };

} // namespace servo
