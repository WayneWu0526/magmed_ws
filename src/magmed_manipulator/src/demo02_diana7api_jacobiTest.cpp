#include <DianaAPIDef.h>
#include <DianaAPI.h>
#include <cstring>
#include <iostream>
#include <cmath>
#include <eigen3/Eigen/Dense>
#define JOINT_NUM 7

void logRobotState(StrRobotStateInfo *pinfo, const char *strIpAddress) // Heart beat server
{
    strIpAddress = "192.168.10.75";
    static int staCnt = 1;
    if ((staCnt++ % 1000 == 0) && pinfo)
    {
        for (int i = 0; i < JOINT_NUM; ++i)
        {
            printf("jointPos[%d] = %f \n", i, pinfo->jointPos[i]);
            printf("jointCurrent [%d] = %f \n", i, pinfo->jointCurrent[i]);
            printf("jointTorque [%d] = %f \n", i, pinfo->jointTorque[i]);
            if (i < 6)
            {
                printf("tcpPos [%d] = %f \n", i, pinfo->tcpPos[i]); // Tool Center Point;
            }
        }
    }
}

void errorControl(int e, const char *strIpAddress)
{
    strIpAddress = "192.168.10.75";
    const char *strError = formatError(e); // 该函数后面会介绍
    printf("error code (%d):%s\n", e, strError);
}

int main(int argc, char *argv[])
{
    // 初始化 API，完成其他功能函数使用前的初始化准备工作。
    const char *strIpAddress = "192.168.10.75";
    srv_net_st *pinfo = new srv_net_st();
    memset(pinfo->SrvIp, 0x00, sizeof(pinfo->SrvIp));
    memcpy(pinfo->SrvIp, "192.168.10.75", strlen("192.168.10.75"));
    pinfo->LocHeartbeatPort = 0;
    pinfo->LocRobotStatePort = 0;
    pinfo->LocSrvPort = 0;
    int nRet = 0;

    do
    {
        nRet = initSrv(errorControl, logRobotState, pinfo); // 机械臂的心跳服务会一直向上位机发送信号，可以屏蔽
        if (nRet < 0)
        {
            printf("192.168.10.75 initSrv failed! nReturn value = %d\n", nRet);
            break;
        }
        if (pinfo)
        {
            delete pinfo;
            pinfo = nullptr;
        }
        // main
        double matrix_jacobi[6 * JOINT_NUM] = {0.0};
        const char *strIpAddress = "192.168.10.75";
        nRet = getJacobiMatrix(matrix_jacobi, strIpAddress);
        if (nRet < 0)
        {
            printf("getJacobiMatrix failed!\n");
        }
        else
        {
            for (int i = 0; i < 6; ++i)
            {
                for (int j = 0; j < JOINT_NUM; ++j)
                {
                    printf("%lf,", matrix_jacobi[i * JOINT_NUM + j]);
                }
                printf("\n");
            }
        }

        const int intJointCount = JOINT_NUM;
        const int intTcpCount = 6;
        double dblJacobiMatrix[intTcpCount * intJointCount] = {0};
        double dblJointPosition[intJointCount] = {0.0, 0.0, 0.0, M_PI/2.0, 0.0, 0.0, 0.0};
        int ret = calculateJacobi(dblJacobiMatrix, dblJointPosition, intJointCount, strIpAddress);
        if (ret == -1)
        {
            printf("cannot get jacobi matrix");
        }
        else
        {
            for (int i = 0; i < intTcpCount; ++i)
            {
                for (int j = 0; j < intJointCount; ++j)
                {
                    printf("%lf,", dblJacobiMatrix[i * intTcpCount + j]);
                }
                printf("\n");
            }
        }

        double default_tcp[6] = {0.0};
        nRet = getDefaultActiveTcpPose(default_tcp, strIpAddress);
        if (nRet < 0)
        {
            printf("getDefaultActiveTcpPose failed!\n");
        }
        ret = calculateJacobiTF(dblJacobiMatrix, dblJointPosition, intJointCount,
                                default_tcp, strIpAddress);
        if (ret == -1)
        {
            printf("cannot get jacobi matrix");
        }
        else
        {
            for (int i = 0; i < intTcpCount; ++i)
            {
                for (int j = 0; j < intJointCount; ++j)
                {
                    printf("%lf,", dblJacobiMatrix[i * intTcpCount + j]);
                }
                printf("\n");
            }
        }

        Eigen::Matrix<double, intTcpCount, intJointCount> jacobi;
        // convert dblJacobiMatrix to jacobi
        for (int i = 0; i < intTcpCount; ++i)
        {
            for (int j = 0; j < intJointCount; ++j)
            {
                jacobi(i, j) = dblJacobiMatrix[i * intTcpCount + j];
            }
        }
        // create joint vector
        Eigen::Matrix<double, intJointCount, 1> joint;
        joint << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        // create twist vector
        Eigen::Matrix<double, intTcpCount, 1> twist;
        twist = jacobi * joint;
        // print twist
        std::cout << "twist: " << twist << std::endl;


    } while (0);

    destroySrv(strIpAddress);

    return 0;
}