/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       remote_control.c/h
  * @brief      遥控器处理，遥控器是通过类似SBUS的协议传输，利用DMA传输方式节约CPU
  *             资源，利用串口空闲中断来拉起处理函数，同时提供一些掉线重启DMA，串口
  *             的方式保证热插拔的稳定性。
  * @note       该任务是通过串口中断启动，不是freeRTOS任务
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-01-2019     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "remote_control.h"
#include "main.h"
#include "CAN_receive.h"
//#include "usart.h"
//#include "dma.h"
#include "bsp_rc.h"

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;

static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);

RC_ctrl_t rc_ctrl;
static uint8_t sbus_rx_buf[2][RC_FRAME_LENGTH];
uint8_t remote_ready = 0;//遥控器准备完成 

void remote_control_init(void)
{
	RC_init(sbus_rx_buf[0], sbus_rx_buf[1], RC_FRAME_LENGTH);
}


//串口中断
void USART3_IRQHandler(void)
{
	//have received data  ???
	if(huart3.Instance->SR & UART_FLAG_RXNE)
	{
		__HAL_UART_CLEAR_FEFLAG(&huart3);
	}
	else if(USART3->SR & UART_FLAG_IDLE)
	{
	
    static uint16_t this_time_rx_len = 0;
		__HAL_UART_CLEAR_PEFLAG(&huart3);
		
		if( (hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET)
		{
			//current memory buffer used is memory0
			
			//disable dma to change dma register
			__HAL_DMA_DISABLE(&hdma_usart3_rx);
			
			//get received data length, length = set_data_length - remain_length 
			this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

			//reset set_data_length
			hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;
			
			//change memory0 to memory1
			hdma_usart3_rx.Instance->CR |= DMA_SxCR_CT;
			
			//enable dma
			__HAL_DMA_ENABLE(&hdma_usart3_rx);
			
			//1 frame length is correct data
			if(this_time_rx_len == RC_FRAME_LENGTH)
			{
				sbus_to_rc(sbus_rx_buf[0], &rc_ctrl);
			}
		}
			else
			{
				__HAL_DMA_DISABLE(&hdma_usart3_rx);
				
				this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;
				
				hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;
				
				//change memory1 to memory0
				DMA1_Stream1->CR &= ~(DMA_SxCR_CT);
				
				__HAL_DMA_ENABLE(&hdma_usart3_rx);
				
				if(this_time_rx_len == RC_FRAME_LENGTH)
				{
					sbus_to_rc(sbus_rx_buf[1], &rc_ctrl);
				}
			}
	}
}

static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
{
    if (sbus_buf == NULL || rc_ctrl == NULL)
    {
        return;
    }
		
		rc_ctrl->ch[1] = (sbus_buf[1] | (sbus_buf[2] << 8)) & 0x07ff;        //Channel 1
    rc_ctrl->ch[2] = ((sbus_buf[2] >> 3) | (sbus_buf[3] << 5)) & 0x07ff; //Channel 2
    rc_ctrl->ch[0] = ((sbus_buf[3] >> 6) | (sbus_buf[4] << 2) |          //Channel 3
                         (sbus_buf[5] << 10)) &0x07ff;
    rc_ctrl->ch[3] = ((sbus_buf[5] >> 1) | (sbus_buf[6] << 7)) & 0x07ff; //Channel 4
    
		rc_ctrl->sw[0] = ((int16_t)sbus_buf[6] >> 4 | ((int16_t)sbus_buf[7] << 4 )) & 0x07FF;                   								//Channel 5
    rc_ctrl->sw[1] = ((int16_t)sbus_buf[7] >> 7 | ((int16_t)sbus_buf[8] << 1 ) | (int16_t)sbus_buf[9] << 9 ) & 0x07FF;    	//Channel 6
    rc_ctrl->sw[2] = ((int16_t)sbus_buf[9] >> 2 | ((int16_t)sbus_buf[10] << 6 )) & 0x07FF;;                    							//Channel 7
    rc_ctrl->sw[3] = ((int16_t)sbus_buf[10] >> 5 | ((int16_t)sbus_buf[11] << 3 )) & 0x07FF;                    							//Channel 8
    rc_ctrl->sw[4] = ((int16_t)sbus_buf[12] << 0 | ((int16_t)sbus_buf[13] << 8 )) & 0x07FF;                  								//Channel 9
    rc_ctrl->sw[5] = ((int16_t)sbus_buf[13] >> 3 | ((int16_t)sbus_buf[14] << 5 )) & 0x07FF;                               	//Channel 10
    rc_ctrl->sw[6] = ((int16_t)sbus_buf[14] >> 6 | ((int16_t)sbus_buf[15] << 2 ) | (int16_t)sbus_buf[16] << 10 ) & 0x07FF;	//Channel 11
    rc_ctrl->sw[7] = ((int16_t)sbus_buf[16] >> 1 | ((int16_t)sbus_buf[17] << 7 )) & 0x07FF;                    							//Channel 12

		rc_ctrl->ch[0] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->ch[1] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->ch[2] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->ch[3] -= RC_CH_VALUE_OFFSET;
    

//		//新遥控器
		rc_ctrl->ch[0] += 20;	//y(-694,693)
		rc_ctrl->ch[1] += 20;	//x(-693,694)
		rc_ctrl->ch[2] = -rc_ctrl->ch[2]+144;		//m(-518,843)
		rc_ctrl->ch[3] += 4;		//w(-694,693)
	
    rc_ctrl->sw[2] = map(rc_ctrl->sw[2],306,1694,1694,306);	
    rc_ctrl->sw[3] = map(rc_ctrl->sw[3],306,1694,1694,306);	
		rc_ctrl->ch[1] = map(rc_ctrl->ch[1],656,-656,-700,700);		//x
		rc_ctrl->ch[0] = map(rc_ctrl->ch[0],-800,796,700,-700);	//y
		rc_ctrl->ch[2] = map(rc_ctrl->ch[2],-632,901,25,0);		//m
		rc_ctrl->ch[3] = map(rc_ctrl->ch[3],-820,780,-700,700);  
		rc_ctrl->ch[3] = 0.5*(rc_ctrl->ch[3]);
    rc_ctrl->ch[1] = 0.5*(rc_ctrl->ch[1]);
    rc_ctrl->ch[0] = 0.5*(rc_ctrl->ch[0]);		
		
		//死区(-30,30)
		if(rc_ctrl->ch[0]>-14&&rc_ctrl->ch[0]<10) rc_ctrl->ch[0]=0;              
		if(rc_ctrl->ch[1]>-30&&rc_ctrl->ch[1]<20) rc_ctrl->ch[1]=0;                
		if(rc_ctrl->ch[2]>=0&&rc_ctrl->ch[2]<=3) rc_ctrl->ch[2]=0;              
		if(rc_ctrl->ch[3]>-22&&rc_ctrl->ch[3]<22) rc_ctrl->ch[3]=0;        

    remote_ready = 1;
}                                                                            																																					



int map(int x, int in_min, int in_max, int out_min, int out_max)		//映射函数
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


/*



  																																	
306																							  					  	  306
 sw[]																						 	 					  		sw[7]
1694																							 					  		1694
 
306																														    	306
sw[6]                                                              sw[4]
1694                                                               1694

306         306                                        306        306   
sw[0]      sw[2]    sw[1]:306-1694   sw[5]:306-1694  sw[]1000   sw[3] 
1694       1694                                         1694       1694
                                      710
               688                   1425
                |                      |
                |                      |
   54    -616------ch[3]770   -354---------ch[0] 339  0
                |                      |
                |                      |
	             ch[2]                 ch[1]
               _699                   38   
*/
