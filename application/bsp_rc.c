#include "bsp_rc.h"
#include "main.h"


extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;
void RC_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
	
	//enable the dma transfer for the receiver request
	SET_BIT(huart3.Instance->CR3, USART_CR3_DMAR);
	
	//enable idle interrupt
	__HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
	
	//disable dma, to change the dma register
	__HAL_DMA_DISABLE(&hdma_usart3_rx);
	
	//disable dma again but why?
	//what's the condition?
	while(hdma_usart3_rx.Instance->CR & DMA_SxCR_EN)
	{
		__HAL_DMA_DISABLE(&hdma_usart3_rx);
	}
	
	//??
	hdma_usart3_rx.Instance->PAR = (uint32_t) & (USART3->DR);
	
	//memory buffer 1
	hdma_usart3_rx.Instance->M0AR = (uint32_t)(rx1_buf);
	
	//momory buffer 2
	hdma_usart3_rx.Instance->M1AR = (uint32_t)(rx2_buf);
	
	//data length
	hdma_usart3_rx.Instance->NDTR = dma_buf_num;
	
	//enable double memory buffer
	SET_BIT(hdma_usart3_rx.Instance->CR, DMA_SxCR_DBM);
	
	
	//enable dma
	__HAL_DMA_ENABLE(&hdma_usart3_rx);

}








