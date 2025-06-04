#include "vision.h"



#define VISION_DMA 			DMA1
#define VISIONRX_CH 		LL_DMA_STREAM_1
#define VISIONTX_CH 		LL_DMA_STREAM_2
#define VISION_UART 		USART1


NUCToBoard nucToBoard;
BoardToNUC boardToNuc;

char visionResBuffer[sizeof(NUCToBoard)*2];


void visionInit()
{
	
    LL_USART_EnableDMAReq_RX(VISION_UART);
	//DBUS 串口DMA设置
	LL_DMA_SetMemoryAddress(VISION_DMA, VISIONRX_CH,(uint32_t)visionResBuffer);
	LL_DMA_SetPeriphAddress(VISION_DMA, VISIONRX_CH,(uint32_t)&VISION_UART->RDR);
	LL_DMA_SetDataLength(VISION_DMA, VISIONRX_CH, (uint32_t)sizeof(visionResBuffer));
	LL_DMA_EnableStream(VISION_DMA, VISIONRX_CH);
	
	//DBUS 串口DMA设置
	LL_DMA_SetMemoryAddress(VISION_DMA, VISIONTX_CH,(uint32_t)&boardToNuc);
	LL_DMA_SetPeriphAddress(VISION_DMA, VISIONTX_CH,(uint32_t)&VISION_UART->RDR);
	LL_DMA_SetDataLength(VISION_DMA, VISIONTX_CH, (uint32_t)sizeof(boardToNuc));
	LL_DMA_EnableStream(VISION_DMA, VISIONTX_CH);

    LL_USART_EnableIT_IDLE(VISION_UART);	
}


//接收数据终端回调
void visionCallBack()
{
	if(LL_USART_IsActiveFlag_IDLE(VISION_UART))
	{
	// 清除空闲中断标志
        LL_USART_ClearFlag_IDLE(VISION_UART);
        //LL_USART_ReceiveData8(VISION_UART);

        LL_DMA_DisableStream(VISION_DMA, VISIONRX_CH);
        
		//获取接收到的数据长度
		
		uint16_t len = sizeof(visionResBuffer) - LL_DMA_GetDataLength(VISION_DMA, VISIONRX_CH);
        LL_DMA_SetDataLength(VISION_DMA, VISIONRX_CH,sizeof(visionResBuffer));
		
		
		LL_DMA_ClearFlag_HT1(VISION_DMA);
		LL_DMA_ClearFlag_TC1(VISION_DMA);
		
        LL_DMA_EnableStream(VISION_DMA, VISIONRX_CH);
		
		if(len == sizeof(NUCToBoard) )
		{
			memcpy(&nucToBoard,visionResBuffer,sizeof(NUCToBoard));
		}
		
	}
	
}


//发送数据到NUC
void visionSend()
{
	
	
	LL_DMA_EnableStream(VISION_DMA, VISIONTX_CH);
}