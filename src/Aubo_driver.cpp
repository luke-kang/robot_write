#include "Aubo_driver.h"

driver::driver(const char* paddr, int port, const char* username,
               const char* password) {

    /** 接口调用: 登录 ***/
    ret = robotService.robotServiceLogin(paddr, port, username, password);
    if(ret == aubo_robot_namespace::InterfaceCallSuccCode){
        cout << "登录成功." << endl;
    }
    else{
        cout << "登录失败." << endl;
    }

    /** 如果是连接真实机械臂，需要对机械臂进行初始化　**/
    aubo_robot_namespace::ROBOT_SERVICE_STATE result;

    //工具动力学参数
    aubo_robot_namespace::ToolDynamicsParam toolDynamicsParam;
    memset(&toolDynamicsParam, 0, sizeof(toolDynamicsParam));

    ret = robotService.rootServiceRobotStartup(toolDynamicsParam/**工具动力学参数**/,
                                               6        /*碰撞等级*/,
                                               true     /*是否允许读取位姿　默认为true*/,
                                               true,    /*保留默认为true */
                                               1000,    /*保留默认为1000 */
                                               result); /*机械臂初始化*/

    if(ret != aubo_robot_namespace::InterfaceCallSuccCode){
        std::cerr<<"机械臂初始化失败."<<std::endl;
    }
}

// 初始化机器人。
void driver::init_robot(){
    /** 接口调用: 初始化运动属性 ***/
    robotService.robotServiceInitGlobalMoveProfile();

    /** 接口调用: 设置关节型运动的最大加速度 ***/
    aubo_robot_namespace::JointVelcAccParam jointMaxAcc;
    jointMaxAcc.jointPara[0] = 20.0/180.0*M_PI;
    jointMaxAcc.jointPara[1] = 20.0/180.0*M_PI;
    jointMaxAcc.jointPara[2] = 20.0/180.0*M_PI;
    jointMaxAcc.jointPara[3] = 20.0/180.0*M_PI;
    jointMaxAcc.jointPara[4] = 20.0/180.0*M_PI;
    jointMaxAcc.jointPara[5] = 20.0/180.0*M_PI;   //接口要求单位是弧度
    robotService.robotServiceSetGlobalMoveJointMaxAcc(jointMaxAcc);

    /** 接口调用: 设置关节型运动的最大速度 ***/
    aubo_robot_namespace::JointVelcAccParam jointMaxVelc;
    jointMaxVelc.jointPara[0] = 20.0/180.0*M_PI;
    jointMaxVelc.jointPara[1] = 20.0/180.0*M_PI;
    jointMaxVelc.jointPara[2] = 20.0/180.0*M_PI;
    jointMaxVelc.jointPara[3] = 20.0/180.0*M_PI;
    jointMaxVelc.jointPara[4] = 20.0/180.0*M_PI;
    jointMaxVelc.jointPara[5] = 20.0/180.0*M_PI;   //接口要求单位是弧度
    robotService.robotServiceSetGlobalMoveJointMaxVelc(jointMaxVelc);

    /** 运动到初始位姿 **/
//    double jointAngle[aubo_robot_namespace::ARM_DOF] = {0};
//    initJointAngleArray(jointAngle, 0.0/180.0*M_PI,  0.0/180.0*M_PI,  90.0/180.0*M_PI, 0.0/180.0*M_PI, 90.0/180.0*M_PI, 0.0/180.0*M_PI);
//    ret = robotService.robotServiceJointMove(jointAngle, true);
//    if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
//    {
//        std::cerr<<"运动至零位姿态失败.　ret:"<<ret<<std::endl;
//    }


    /** 接口调用: 初始化运动属性 ***/
    robotService.robotServiceInitGlobalMoveProfile();

    /** 接口调用: 设置末端型运动的最大加速度 　　直线运动属于末端型运动***/
    double lineMoveMaxAcc;
    lineMoveMaxAcc = 0.2;   //单位米每秒
    robotService.robotServiceSetGlobalMoveEndMaxLineAcc(lineMoveMaxAcc);
    robotService.robotServiceSetGlobalMoveEndMaxAngleAcc(lineMoveMaxAcc);

    /** 接口调用: 设置末端型运动的最大速度 直线运动属于末端型运动***/
    double lineMoveMaxVelc;
    lineMoveMaxVelc = 0.2;   //单位米每秒
    robotService.robotServiceSetGlobalMoveEndMaxLineVelc(lineMoveMaxVelc);
    robotService.robotServiceGetGlobalMoveEndMaxAngleVelc(lineMoveMaxVelc);
}

void driver::initJointAngleArray(double *array, double joint0, double joint1,
                                 double joint2, double joint3,double joint4,
                                 double joint5){

    array[0] = joint0;
    array[1] = joint1;
    array[2] = joint2;
    array[3] = joint3;
    array[4] = joint4;
    array[5] = joint5;
}

void driver::movej(double *joints) {
    ret = robotService.robotServiceJointMove(joints, true);

    if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
    {
        cout << "movej移动失败!" << endl;
        return;
    }
}

void driver::movel(double *pos) {
    aubo_robot_namespace::Pos cartPos;
    cartPos.x = pos[0];
    cartPos.y = pos[1];
    cartPos.z = pos[2];

    // 末端的姿态，aubo中需要用的是四元素，所以要转换成四元素才行
    aubo_robot_namespace::Rpy rpy;
    aubo_robot_namespace::Ori ori;

    rpy.rx = pos[3];
    rpy.ry = pos[4];
    rpy.rz = pos[5];

    // 转成四元素
    robotService.RPYToQuaternion(rpy,ori);

    // 设置Base坐标系
    aubo_robot_namespace::CoordCalibrateByJointAngleAndTool userCoord;
    userCoord.coordType = aubo_robot_namespace::BaseCoordinate;

    /** 设置工具端参数 **/
    aubo_robot_namespace::ToolInEndDesc toolDesc;
    toolDesc.toolInEndPosition.x = 0;
    toolDesc.toolInEndPosition.y = 0;
    toolDesc.toolInEndPosition.z = 0;

    // 设置工具末端的姿态
    toolDesc.toolInEndOrientation.x = 0;
    toolDesc.toolInEndOrientation.y = 0;
    toolDesc.toolInEndOrientation.z = 0;
    toolDesc.toolInEndOrientation.w = 1;

    ret = robotService.robotMoveLineToTargetPosition(userCoord,cartPos,toolDesc,true);

    if(ret != aubo_robot_namespace::InterfaceCallSuccCode){
        cout << "movel移动失败" << endl;
        return;
    }else{
        cout << "movel移动成功" << endl;
    }
}
