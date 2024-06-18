#ifndef _CH9326_LIB_H
#define _CH9326_LIB_H
#include <usb.h>
// #include <libusb-1.0/libusb.h>
#ifdef __cplusplus
extern "C" {
#endif

/*********************************************************************  
* 函 数 名: ch9326_find
* 功能描述: 探测ch9326设备，打开设备前必须调用
* 函数说明: 
* 调用函数: usb_init(from libusb.a)
*			usb_find_buses(from libusb.a)
*			usb_find_devices(from libusb.a)
* 全局变量: ch9326[]
* 输    入: 无
* 返    回: 操作成功 : 找到的设备数(>0)，最多支持16个设备同时工作, 操作失败 : 0
* 设 计 者: tech32                         日期: 2011-11
* 修 改 者:                                日期:
* 版    本: VER 1.0.0.0
***********************************************************************/
extern
unsigned char
ch9326_find(
    void
    );

/*********************************************************************
* 函 数 名: ch9326_open
* 功能描述: 打开设备，创建接收数据线程
* 函数说明:
* 调用函数: usb_open(from libusb.a)
* 全局变量: ch9326[]/usb_dev[]
* 输    入: 设备索引号，0对应第1个设备，1对应第2个设备... n对应第n+1个设备(n<=16)
* 返    回: 操作成功 : 返回设备句柄, 操作失败 : NULL
* 设 计 者: tech32                         日期: 2011-11
* 修 改 者:                                日期:
* 版    本: VER 1.0.0.0
***********************************************************************/
extern
usb_dev_handle *
ch9326_open(
    unsigned char index
    );

/*********************************************************************
* 函 数 名: ch9326_close
* 功能描述: 关闭设备
* 函数说明: 
* 调用函数: usb_close(from libusb.a)
* 全局变量: 
* 输    入: 设备索引号，0对应第1个设备，1对应第2个设备... n对应第n+1
			个设备(n<=16)
* 返    回: 
			操作成功 : 1
			操作失败 : 0
* 设 计 者: tech32                         日期: 2011-11
* 修 改 者:                                日期:
* 版    本: VER 1.0.0.0
***********************************************************************/
extern
unsigned char
ch9326_close(
    unsigned char index
    );

/*********************************************************************
* 函 数 名: ch9326_set
* 功能描述: 设置数据通讯格式
* 函数说明: 
* 调用函数: usb_control_msg(from libusb.a)
* 全局变量: 
* 输    入: unsigned char index : 设备索引号，0对应第1个设备，1对应第2个设备... n对应第n+1个设备(n<=16)
*			unsigned char rate : 01 = 300bps, 02 = 600bps, 03 = 1200bps, 04 = 2400bps, 05 = 4800bps, 06 = 9600(default)bps,
*								 07 = 14400bps, 08 = 19200bps, 09 = 28800bps, 10 = 38400bps, 11 = 57600bps, 12 = 76800bps,
*								 13 = 115200bps
*			unsigned char check : 01 = odd, 02 = even, 03 = space, 04 = none(default)
*			unsigned char stop_bits : 01 = 1bit stop bit(default)
*									  02 = 2bit stop bit
*			unsigned char data_bits : 01 = 5bit data bit, 02 = 6bit data bit, 03 = 7bit data bit, 04 = 8bit data bit(default)
*			unsigned char interval :	0x10 = 3ms(default), 0x20 = 6ms, 0x30 = 9ms
* 返    回: 操作成功 : 返回1, 操作失败 : 返回0
* 设 计 者: tech32                         日期: 2011-11
* 修 改 者:                                日期:
* 版    本: VER 1.0.0.0
***********************************************************************/
extern
unsigned char
ch9326_set(
	unsigned char index,
	unsigned char rate,
	unsigned char check,
	unsigned char stop_bits,
	unsigned char data_bits,
	unsigned char interval
	);

/*********************************************************************
* 函 数 名: ch9326_send
* 功能描述: 发送数据
* 函数说明: 
* 调用函数: usb_interrupt_write(from libusb.a)
* 全局变量: 
* 输    入: unsigned char index : 设备索引号，0对应第1个设备，1对应第2个设备... n对应第n+1个设备(n<=16)
			unsigned char *data : 发送数据缓冲区		
			unsigned long length : 发送数据长度
* 返    回: 操作成功 : 返回发送的数据长度, 操作失败 : 返回0
* 设 计 者: tech32                         日期: 2011-11
* 修 改 者:                                日期:
* 版    本: VER 1.0.0.0
***********************************************************************/
extern
unsigned long
ch9326_send(
	unsigned char index,
	unsigned char *data,
	unsigned long length
	);

/*********************************************************************
* 函 数 名: ch9326_recv
* 功能描述: 接收数据
* 函数说明: 
* 调用函数: 
* 全局变量: 
* 输    入: unsigned char index : 设备索引号，0对应第1个设备，1对应第2个设备... n对应第n+1个设备(n<=16)
			unsigned char *data : 接收数据缓冲区
			unsigned long length : 接收数据长度
* 返    回: 操作成功 : 返回发送的数据长度, 操作失败 : 返回0
* 设 计 者: tech32                         日期: 2011-11
* 修 改 者:                                日期:
* 版    本: VER 1.0.0.0
***********************************************************************/
extern
unsigned long
ch9326_recv(
	unsigned char index,
    char *data,
	unsigned long length
	);

/*********************************************************************
* 函 数 名: ch9326_set_gpiodata
* 功能描述:
* 函数说明:
* 调用函数: usb_control_msg(from libusb.a)
* 全局变量:
* 输    入: unsigned char index : 设备索引号，0对应第1个设备，1对应第2个设备... n对应第n+1个设备(n<=16)
*			unsigned char data : 位0用来设置IO1电平，0为输出低电平、1为输出高电平
*								 位1用来设置IO2电平，0为输出低电平、1为输出高电平
* 返    回:
            操作成功 : 1
            操作失败 : 0
* 设 计 者: tech32                         日期: 2011-11
* 修 改 者: tech33                         日期: 2014-11
* 版    本: VER 1.0.0.0
***********************************************************************/
extern unsigned char
ch9326_set_gpiodata(
    unsigned char index,
    unsigned char data
    );

/*********************************************************************
* 函 数 名: ch9326_set_gpiodir
* 功能描述:
* 函数说明:
* 调用函数: usb_control_msg(from libusb.a)
* 全局变量:
* 输    入: unsigned char index : 设备索引号，0对应第1个设备，1对应第2个设备... n对应第n+1个设备(n<=16)
*			unsigned char dir : 位0用来设置IO1方向，0为输入、1为输出
*						 		位1用来设置IO2方向，0为输入、1为输出
* 返    回:
            操作成功 : 1
            操作失败 : 0
* 设 计 者: tech32                         日期: 2011-11
* 修 改 者: tech33                         日期: 2014-11
* 版    本: VER 1.0.0.0
***********************************************************************/
extern unsigned char
ch9326_set_gpiodir(
    unsigned char index,
    unsigned char dir
    );

/*********************************************************************
* 函 数 名: ch9326_set_gpio
* 功能描述:
* 函数说明:
* 调用函数: usb_control_msg(from libusb.a)
* 全局变量:
* 输    入: unsigned char index : 设备索引号，0对应第1个设备，1对应第2个设备... n对应第n+1个设备(n<=16)
*			unsigned char dir : 位0用来设置IO1方向，0为输入、1为输出
*						 		位1用来设置IO2方向，0为输入、1为输出
*			unsigned char data : 位0用来设置IO1电平，0为输出低电平、1为输出高电平
*								 位1用来设置IO2电平，0为输出低电平、1为输出高电平
* 返    回:
            操作成功 : 1
            操作失败 : 0
* 设 计 者: tech32                         日期: 2011-11
* 修 改 者:                                日期:
* 版    本: VER 1.0.0.0
***********************************************************************/
extern unsigned char
ch9326_set_gpio(
    unsigned char index,
    unsigned char dir,
    unsigned char data
    );

/*********************************************************************
* 函 数 名: ch9326_get_gpio
* 功能描述:
* 函数说明:
* 调用函数: usb_control_msg(from libusb.a)
* 全局变量:
* 输    入: unsigned char index : 设备索引号，0对应第1个设备，1对应第2个设备... n对应第n+1个设备(n<=16)
*			unsigned char data : 位5表示IO1的输入电平状态，为1则表示高电平，为0表示低电平
                                 位3表示IO2的输入电平状态，为1则表示高电平，为0表示低电平
* 返    回:
            操作成功 : 1
            操作失败 : 0
* 设 计 者: tech32                         日期: 2011-11
* 修 改 者:                                日期:
* 版    本: VER 1.0.0.0
***********************************************************************/
extern unsigned char
ch9326_get_gpio(
    unsigned char index,
    char *data
    );

/*********************************************************************
* 函 数 名: ch9326_connected
* 功能描述: 接收数据
* 函数说明:
* 调用函数:
* 全局变量:
* 输    入: unsigned char index : 设备索引号，0对应第1个设备，1对应第2个设备... n对应第n+1个设备(n<=16)
* 返    回: 操作成功 : 返回1, 操作失败 : 返回0
* 设 计 者: tech33                        日期: 2014-11
* 修 改 者:                                      日期:
* 版    本: VER 1.0.0.0
***********************************************************************/
extern unsigned char ch9326_connected(unsigned char index);

#ifdef __cplusplus
}
#endif

#endif // _CH9326_LIB_H
