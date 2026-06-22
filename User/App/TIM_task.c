//
// Created by 36301 on 2026/6/14.
//

#include "TIM_task.h"
#include "All_Init.h"
#include "stm32f4xx_hal_tim.h"
#include "VOFA.h"
#include "FreeRTOS.h"
#include "task.h"

/* xPortSysTickHandler 在 port.c 中定义，未在公开头文件中声明 */
extern void xPortSysTickHandler(void);
/*DBUS->Remote.CH0  = (int16_t)frame->channel0 - 1024;
DBUS->Remote.CH1  = (int16_t)frame->channel1 - 1024;
DBUS->Remote.CH2  = (int16_t)frame->channel2 - 1024;
DBUS->Remote.CH3  = (int16_t)frame->channel3 - 1024;*/
//怀疑是把任务放在了定时器中断的原因
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

    if (htim->Instance == TIM2) {//用于Freetos + GM6020电机控制
        HAL_IncTick();
        if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
            xPortSysTickHandler();
        }

        static uint32_t tick_cnt = 0;
        tick_cnt++;
        if (tick_cnt % 1 == 0) {  /* 1kHz 发送 */
           // DJI_Current_Ctrl(&hcan2, 0x1FE, 5000, 5000, 5000, 5000);

            DJI_Current_Ctrl(&hcan1, 0x1FE, 5000, 5000, 5000, 5000);
        }
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
        DM_Motor_Send(&hcan2, 0x1FE, 0, 0,0, 0);//电机是0X1FE
    }

}



