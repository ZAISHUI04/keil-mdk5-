#include "main.h"
#include "usart.h"
#include "struct_typedef.h"
#include "vofa.h"
float vofa_send[6];



extern UART_HandleTypeDef huart6;

//发送函数
void vofa_tx_main(float *data)
{
  float fdata[4]={0};
	uint8_t tail[4];
	tail[0]=0x00;
	tail[1]=0x00;
	tail[2]=0x80;
	tail[3]=0x7f;
	/*在下面添加发送的数据*/
	fdata[0] = data[0];
	fdata[1] = data[1];
	fdata[2] = data[2];
	fdata[3] = data[3];
	
	
	/*在下面使用对应的串口发送函数*/
     HAL_UART_Transmit(&huart6,( uint8_t *)fdata,sizeof(fdata), 1000);
     HAL_UART_Transmit(&huart6,tail,4,1000);
     HAL_Delay (1);
}


