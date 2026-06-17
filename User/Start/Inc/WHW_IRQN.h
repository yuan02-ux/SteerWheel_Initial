#ifndef __WHW_IRQN_H
#define __WHW_IRQN_H

#include "All_Init.h"
#include "Vision.h"
//轮电机
#define M3508_1left 0x201
#define M3508_2left 0x202
#define M3508_2right 0x203
#define M3508_1right 0x204
//yaw电机
#define GM6020_1left 0x201
#define GM6020_2left 0x202
#define GM6020_2right 0x203
#define GM6020_1right 0x204
extern void BSP_TIM_IRQHandler(TIM_HandleTypeDef *htim);
extern void BSP_UART_IRQHandler(UART_HandleTypeDef *huart);
void DWT_DelayUs(uint32_t us);
extern float dt_pc ; 

#endif
