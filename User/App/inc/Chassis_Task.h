#ifndef __CHASSIS_TASK_H
#define __CHASSIS_TASK_H

#include "DJI_Motor.h"
#include "DM_Motor.h"
#include "DBUS.h"
#include "MY_define.h"
#include "RUI_ROOT_INIT.h"
#include "Motors.h"
#include "Power_Ctrl.h"
typedef struct
{
    int16_t Vx; // 上一个角度值
    int16_t Vy;  // 现在的角度值
    int16_t Vw; // 上一个速度值
    int16_t k;  // 现在的速度值

}STEERING_ENGINE;
#endif
