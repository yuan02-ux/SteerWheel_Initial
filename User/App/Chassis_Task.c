#include "Chassis_Task.h"
#include "math.h"
#include "DBUS.h"
#include "DJI_Motor.h"
//DJI_Current_Ctrl
/*DBUS->Remote.CH0  = (int16_t)frame->channel0 - 1024;
DBUS->Remote.CH1  = (int16_t)frame->channel1 - 1024;
DBUS->Remote.CH2  = (int16_t)frame->channel2 - 1024;
DBUS->Remote.CH3  = (int16_t)frame->channel3 - 1024;*/
/*typedef struct
{
    int8_t ONLINE_JUDGE_TIME;
    int16_t Angle_last; // 上一个角度值
    int16_t Angle_now;  // 现在的角度值
    int16_t Speed_last; // 上一个速度值
    int16_t Speed_now;  // 现在的速度值
    int16_t acceleration;//加速度
    int16_t conEncode;//连续编码值
    int16_t current;
    int8_t temperature;
    int32_t Angle_Infinite;/////连续的角度值
    int64_t Stuck_Time;
    uint16_t Stuck_Flag[2];
    int16_t Laps;
    float Error;
    float Aim;
    float Aim_last;
    float dt;
}DJI_MOTOR_DATA_Typedef;
typedef struct
{
    int16_t Vx; // 上一个角度值
    int16_t Vy;  // 现在的角度值
    int16_t Vw; // 上一个速度值
    int16_t k;  // 现在的速度值

}STEERING_ENGINE;
*/
DJI_MOTOR_DATA_Typedef M3508;
DJI_MOTOR_DATA_Typedef GM6020;
STEERING_ENGINE chassis;
extern DBUS_Typedef DBUS;
void Chassis_pingyi_speed(DJI_MOTOR_DATA_Typedef *motor)//M3508
{//左侧负责平移，x轴为通道2，y轴为通道3
    chassis.Vx =DBUS.Remote.CH2;
    chassis.Vy =DBUS.Remote.CH3;
    M3508.SPEED =hypot(chassis.Vx,chassis.Vy );
}
void Chassis_pingyi_angel()//GM6020
{
    chassis.Vx =DBUS.Remote.CH2;
    chassis.Vy =DBUS.Remote.CH3;
    GM6020.ANGEL= atan2(chassis.Vy, chassis.Vx);
}