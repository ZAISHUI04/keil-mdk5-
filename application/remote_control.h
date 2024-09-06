/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       remote_control.c/h
  * @brief      遥控器处理，遥控器是通过类似SBUS的协议传输，利用DMA传输方式节约CPU
  *             资源，利用串口空闲中断来拉起处理函数，同时提供一些掉线重启DMA，串口
  *             的方式保证热插拔的稳定性。
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */
#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H
#include "struct_typedef.h"
#include "bsp_rc.h"

#define SBUS_RX_BUF_NUM 50u

#define RC_FRAME_LENGTH 25u

#define RC_CH_VALUE_OFFSET  ((uint16_t)1024)

typedef  struct
{
	int16_t ch[4];
  int16_t sw[8]; 

}__attribute__((packed)) RC_ctrl_t;
extern RC_ctrl_t rc_ctrl;

extern void remote_control_init(void);
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);
int map(int x, int in_min, int in_max, int out_min, int out_max);		//映射函数


#endif
