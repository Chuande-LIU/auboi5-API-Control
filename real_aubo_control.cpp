
#include "aubo_driver/aubo_driver.h"
#include "aubo_driver.cpp"
#include <ros/ros.h>

using namespace aubo_driver;

//关节限制
#define MAX_JOINT_ACC 100.0/180.0*M_PI  //unit rad/s^2
#define MAX_JOINT_VEL 50.0/180.0*M_PI   //unit rad/s
#define MAX_END_ACC    4                // unit m/s^2
#define MAX_END_VEL    2                // unit m/s

//自定义关节点，ARM_DOF=6
double zero_poeition[ARM_DOF] = {0}; //关节零位
double initial_poeition[ARM_DOF] = {0.0/180.0*M_PI,  0.0/180.0*M_PI,  90.0/180.0*M_PI, 0.0/180.0*M_PI, 90.0/180.0*M_PI, 0.0/180.0*M_PI};
double postion1[ARM_DOF] = {0.0/180.0*M_PI,  0.0/180.0*M_PI,  90.0/180.0*M_PI, 0.0/180.0*M_PI, 90.0/180.0*M_PI,   0.0/180.0*M_PI};
double postion2[ARM_DOF] = {15.0/180.0*M_PI,  0.0/180.0*M_PI,  90.0/180.0*M_PI, 0.0/180.0*M_PI, 90.0/180.0*M_PI,   0.0/180.0*M_PI};

double postion3[ARM_DOF] = {0.0/180.0*M_PI,  0.0/180.0*M_PI,  45.0/180.0*M_PI, 0.0/180.0*M_PI, 90.0/180.0*M_PI, 0.0/180.0*M_PI};
double postion4[ARM_DOF] = {30.0/180.0*M_PI,  0.0/180.0*M_PI,  90.0/180.0*M_PI, 0.0/180.0*M_PI, 90.0/180.0*M_PI, 0.0/180.0*M_PI};


//192.168.1.8  255.255.255.0  192.168.1.1 本地ipv4设置

void MoveJ(AuboDriver &robot_driver)
{
    

    /** Robot move to zero position **/
    int ret = robot_driver.robot_send_service_.robotServiceJointMove(zero_poeition, true);
    if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
        ROS_ERROR("Failed to move to zero postions, error code:%d", ret);

    //更改关节
    zero_poeition[0] -= 0.7;
    ret = robot_driver.robot_send_service_.robotServiceJointMove(zero_poeition, true);
    if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
        ROS_ERROR("Failed to move to postion2, error code:%d", ret);

    //更改关节
    zero_poeition[0] += 0.7;
    ret = robot_driver.robot_send_service_.robotServiceJointMove(zero_poeition, true);
    if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
        ROS_ERROR("Failed to move to postion2, error code:%d", ret);
    

    //robotServiceJointMove(position, true) 关节空间直线运动
    //robotServiceLineMove(position, true) 笛卡尔空间直线运动
    return;

}

void MoveE(AuboDriver &robot_driver){

    double io_flag_delay_ = 0.5;
    double move_time_ = 2;

    //IO enum类型可在 aubo_driver/include/aubo_driver/AuboRobotMetaType.h 查看
    //24V电压
    robot_driver.robot_send_service_.robotServiceSetToolPowerVoltageType(aubo_robot_namespace::OUT_24V);
    //使0、1口为输出口，并设置为低电平
    robot_driver.robot_send_service_.robotServiceSetToolDigitalIOType(aubo_robot_namespace::TOOL_DIGITAL_IO_0, aubo_robot_namespace::IO_OUT);
    ros::Duration(io_flag_delay_).sleep();
    robot_driver.robot_send_service_.robotServiceSetToolDOStatus(aubo_robot_namespace::TOOL_DIGITAL_IO_0, aubo_robot_namespace::IO_STATUS_VALID);
    ros::Duration(io_flag_delay_).sleep();
    robot_driver.robot_send_service_.robotServiceSetToolDigitalIOType(aubo_robot_namespace::TOOL_DIGITAL_IO_1, aubo_robot_namespace::IO_OUT);
    ros::Duration(io_flag_delay_).sleep();
    robot_driver.robot_send_service_.robotServiceSetToolDOStatus(aubo_robot_namespace::TOOL_DIGITAL_IO_1, aubo_robot_namespace::IO_STATUS_VALID);
    ros::Duration(io_flag_delay_).sleep();
    //0口设置为高点平，夹取
    robot_driver.robot_send_service_.robotServiceSetToolDOStatus(aubo_robot_namespace::TOOL_DIGITAL_IO_0, aubo_robot_namespace::IO_STATUS_INVALID);
    ros::Duration(move_time_).sleep();
    //0口设置为低点平，停止
    robot_driver.robot_send_service_.robotServiceSetToolDOStatus(aubo_robot_namespace::TOOL_DIGITAL_IO_0, aubo_robot_namespace::IO_STATUS_VALID);
    ros::Duration(move_time_).sleep();
    //1口设置为高点平，张开
    robot_driver.robot_send_service_.robotServiceSetToolDOStatus(aubo_robot_namespace::TOOL_DIGITAL_IO_1, aubo_robot_namespace::IO_STATUS_INVALID);
    ros::Duration(move_time_).sleep();
    //1口设置为低点平，停止
    robot_driver.robot_send_service_.robotServiceSetToolDOStatus(aubo_robot_namespace::TOOL_DIGITAL_IO_1, aubo_robot_namespace::IO_STATUS_VALID);
    ros::Duration(move_time_).sleep();

    return;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "real_aubo_control");
    ros::NodeHandle n;

    AuboDriver robot_driver; //实例化对象
    bool ret = robot_driver.connectToRobotController();  //连接机械臂

    /** If connect to a real robot, then you need initialize the dynamics parameters　**/
    aubo_robot_namespace::ROBOT_SERVICE_STATE result;
    //tool parameters
    aubo_robot_namespace::ToolDynamicsParam toolDynamicsParam;
    memset(&toolDynamicsParam, 0, sizeof(toolDynamicsParam));

    robot_driver.robot_send_service_.rootServiceRobotStartup(toolDynamicsParam/**tool dynamics paramters**/,
                                                6        /*collision class*/,
                                                true     /* Is allowed to read robot pose*/,
                                                true,    /*default */
                                                1000,    /*default */
                                                result); /*initialize*/
    /** Initialize move properties ***/
    robot_driver.robot_send_service_.robotServiceInitGlobalMoveProfile();

    /** Set Max joint acc and vel***/
    aubo_robot_namespace::JointVelcAccParam jointMaxAcc;
    aubo_robot_namespace::JointVelcAccParam jointMaxVelc;
    for(int i = 0; i < ARM_DOF; i++)
    {
        jointMaxAcc.jointPara[i] = MAX_JOINT_ACC;
        jointMaxVelc.jointPara[i] = MAX_JOINT_VEL;
    }
    robot_driver.robot_send_service_.robotServiceSetGlobalMoveJointMaxAcc(jointMaxAcc);
    robot_driver.robot_send_service_.robotServiceSetGlobalMoveJointMaxVelc(jointMaxVelc);


    if(ret)
    {
    //关节运动
    MoveJ(robot_driver);
    //夹爪运动
    MoveE(robot_driver);
    }
    else
        ROS_INFO("Failed to connect to the robot controller");

    return 0;
}
