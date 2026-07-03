git add README.md
git commit -m "添加项目说明文档"
git push
STM32F407 四轮舵轮底盘控制程序

本项目是基于大疆开发板 C 型（STM32F407IG）开发的四轮舵轮（SwerveDrive）底盘控制系统。实现了遥控器输入解析、底盘运动学统一解算、舵机零偏校准、舵向电机就近转向算法、以及双环 PID 闭环控制，底盘控制逻辑运行在FreeRTOS 任务中。

开发环境与硬件依赖

1. 软件环境

  - IDE / 工具链：CLion (CMake) + MinGW / arm-none-eabi-gcc
  - 底层配置：STM32CubeMX (HAL 库)
  - 调试与烧录：DepLink / Ozone + OpenOCD / VOFA+ 上位机

2. 硬件架构与 CAN 总线分配

由于单路 CAN 总线带宽限制，切勿将 8 个电机同时挂在同一路 CAN 上（否则在 1KHz 接收频率下极易产生总线拥堵，导致标识符最大的 0x208
舵电机数据丢包、接收无效）。目前总线分配如下：

  - CAN2：挂载 4 个 M3508 驱动轮电机 (反馈 ID: 0x201 ~ 0x204)
  - CAN1：挂载 4 个 GM6020 舵向轮电机 (反馈 ID: 0x205 ~ 0x208)
  - 遥控器：大疆接收机（挂载于 USART3，使用 DBUS 协议解析）

 核心文件目录

C_Board_Framework_Old1/
├── Core/               # HAL 库底层初始化及中断外设配置
├── User/
│   ├── App/            # 业务逻辑层
│   │   ├── Chassis_Task.c   # 底盘运动学统一解算与控制任务
│   │   └── Chassis_Task.h
│   └── Bsp/            # 硬件驱动层
│       ├── bsp_can.c        # CAN 接收中断回调与发送配置
│       └── WHW_IRQN.c       # 中断处理（接收回调逻辑在此定义）

 核心算法与逻辑

1. 运动学统一解算

底盘的平移矢量与自转角速度解算统一在 Chassis_jiesuan 中处理，避免了平移与旋转分离计算导致就近转向时合速度相位丢失的问题。

void Chassis_jiesuan(DJI_MOTOR_DATA_Typedef *M3508_demo, DJI_MOTOR_DATA_Typedef *GM6020_demo, int wheel_id)
{
    // 1. 获取遥控器输入，并转化为底盘平移速度 Vx、Vy 与自转速度 Vw
    chassis.Vx = DBUS.Remote.CH2 * 6; // x轴
    chassis.Vy = DBUS.Remote.CH3 * 6; // y轴
    chassis.Vw = DBUS.Remote.CH0;     // 自转角速度
    
    // 2. 根据 wheel_id 针对四个轮子分别进行速度与角度矢量合成 ...
}

2. 舵轮就近转向逻辑

为防止舵电机在旋转跨度大于 90° 时进行长距离旋转，解算中引入了就近转向算法：

  - 当目标角度与当前角度的偏差大于 90° 时，使目标角度偏转 -180° 或 +180°，同时将 M3508
    驱动轮电机的控制速度取反，从而提高底盘整体的动态响应速度。

舵轮零偏校准与调试指南

新车装配或拆卸舵机后，必须重新校准 GM6020 电机的物理零偏，否则解算角度将出现严重偏差。

1. 零偏读取与写入

1.  将车体架高（使四个轮子悬空），手动将四个舵轮物理方向调整至绝对朝前（车头方向）。
2.  连接 DepLink 仿真器，在 debug 状态下，在线读取四个 GM6020 电机当前的反馈转子角度值（ecd）。
3.  将读取到的零偏值依次写入 Chassis_Task.c 中的 GM6020_OFFSET 数组中进行物理校准：
    // 对应 1 号 ~ 4 号舵电机的零偏机械角度反馈值
    const int16_t GM6020_OFFSET[5] = {0, 1350, 1350, 5446, 5446}; 

2. PID 调参指南

  - GM6020 舵向电机：采用“外环角度 + 内环速度”的双环 PID 进行位置跟踪，优先整定内环速度环 Kp，确保响应无震荡后，再引入外环角度 Kp。
  - M3508 驱动电机：采用速度闭环控制，Kp调的不好的时候，底盘容易走歪或走直线跑偏。

快速上手编译

1.  使用 CLion 打开本项目。
2.  检查开发环境中的编译器路径（通常为 arm-none-eabi-gcc）。
3.  点击 CLion 顶部的编译按钮进行 Build。
4.  使用  DepLink 烧录到主控板，复位运行。
