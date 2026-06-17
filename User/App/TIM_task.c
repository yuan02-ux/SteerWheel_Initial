//
// Created by 36301 on 2026/6/14.
//

#include "TIM_task.h"
#include "All_Init.h"
#include "stm32f4xx_hal_tim.h"
#include "VOFA.h"
/*DBUS->Remote.CH0  = (int16_t)frame->channel0 - 1024;
DBUS->Remote.CH1  = (int16_t)frame->channel1 - 1024;
DBUS->Remote.CH2  = (int16_t)frame->channel2 - 1024;
DBUS->Remote.CH3  = (int16_t)frame->channel3 - 1024;*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

    if (htim->Instance == TIM2) {//用于Freetos
        HAL_IncTick();
    }
    if (htim->Instance == TIM9) { // 云台任务
        VOFA_justfloat((float)DBUS.Remote.CH0,
                (float)DBUS.Remote.CH1,
                (float)DBUS.Remote.CH2,
                (float)DBUS.Remote.CH3,
                0,
                0,
                0,
                0,
                0,
                0);
        DM_Motor_Send(&hcan1, 0x200, 0, 0, 0, 0);
        DM_Motor_Send(&hcan2, 0x1FF, 0, 0,0, 0);
    }

}



