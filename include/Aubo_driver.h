#ifndef ROBOT_WRITE_AUBO_DRIVER_H
#define ROBOT_WRITE_AUBO_DRIVER_H

#include <iostream>
#include <string.h>
#include <math.h>

#include "AuboRobotMetaType.h"
#include "serviceinterface.h"

using namespace std;

class driver{
public:
    driver(const char* paddr, int port, const char* username,
           const char* password);
    ~driver(){}

    // 初始化机器人。
    void init_robot();

    void initJointAngleArray(double *array, double joint0, double joint1,
                             double joint2, double joint3, double joint4,
                             double joint5);

    // 机器人移动。
    void movej(double* joints);
    void movel(double* pos);


private:
    ServiceInterface robotService;
    int ret = aubo_robot_namespace::InterfaceCallSuccCode;

};

#endif //ROBOT_WRITE_AUBO_DRIVER_H
