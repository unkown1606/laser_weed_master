

#include "dbus.h"

#define REMOTE_DEV SBUS

#define DBUS_DMA 		DMA1
#define DBUS_CH 		LL_DMA_STREAM_0
#define DBUS_UART 		USART2

/**************************
		ģʽ2
	ch3			ch2
	  |			 |
ch4	����+����	ch1����+����
	  |			 |
**************************/


FlyskyKey rc;

uint8_t sbusRxBuffer[RC_FRAME_LENGTH]; // DMA�����������
uint8_t sbusRxBuf[RC_FRAME_LENGTH*2]; // DMA������
void sbusHandleData(volatile const uint8_t *SBUS_buf);
void dbusHandleData(volatile const uint8_t *SBUS_buf);
int16_t dbusCH[SBUS_RX_BUF_NUM]; // ��ǰͨ��ֵ


void dbusInit()
{

    LL_USART_EnableDMAReq_RX(DBUS_UART);
	//DBUS ����DMA����
	LL_DMA_SetMemoryAddress(DBUS_DMA, DBUS_CH,(uint32_t)sbusRxBuf);
	LL_DMA_SetPeriphAddress(DBUS_DMA, DBUS_CH,(uint32_t)&DBUS_UART->RDR);
	LL_DMA_SetDataLength(DBUS_DMA, DBUS_CH, (uint32_t)sizeof(sbusRxBuf));
	LL_DMA_EnableStream(DBUS_DMA, DBUS_CH);
    LL_USART_EnableIT_IDLE(DBUS_UART);
	LL_USART_Enable(DBUS_UART);
	
	
//    LL_USART_EnableIT_RXNE_RXFNE(DBUS_UART);
}
uint16_t dmalen;

void dbusCallBack()
{
	if(LL_USART_IsActiveFlag_ORE(DBUS_UART))
	{
		// ��������жϱ�־
        LL_USART_ClearFlag_ORE(DBUS_UART);
        LL_USART_ReceiveData8(DBUS_UART);
	}
	if(LL_USART_IsActiveFlag_IDLE(DBUS_UART))
	{
	// ��������жϱ�־
        LL_USART_ClearFlag_IDLE(DBUS_UART);
        LL_USART_ReceiveData8(DBUS_UART);

        LL_DMA_DisableStream(DBUS_DMA, DBUS_CH);
        
		//��ȡ���յ������ݳ���
		
		dmalen = sizeof(sbusRxBuf) - LL_DMA_GetDataLength(DBUS_DMA, DBUS_CH);
		
		LL_DMA_ClearFlag_HT0(DBUS_DMA);
		LL_DMA_ClearFlag_TC0(DBUS_DMA);
		LL_DMA_SetCurrentTargetMem(DBUS_DMA,DBUS_CH,LL_DMA_CURRENTTARGETMEM0);
        LL_DMA_SetDataLength(DBUS_DMA, DBUS_CH,sizeof(sbusRxBuf));
        LL_DMA_EnableStream(DBUS_DMA, DBUS_CH);
		
		if(dmalen == RC_FRAME_LENGTH)
		{
            memcpy(sbusRxBuffer, sbusRxBuf, dmalen);
			sbusHandleData(sbusRxBuffer);
		}
		else if(dmalen == 18 )
		{
			
            memcpy(sbusRxBuffer, sbusRxBuf, dmalen);
			dbusHandleData(sbusRxBuffer);
			
		}
	}
	
}


uint32_t dbusResTime=0;
uint32_t dbusResTimeLast=0;
uint32_t dbusResTimeErr=0;


//���ݴ���
void sbusHandleData(volatile const uint8_t *SBUS_buf)
{
	dbusResTime = sysTickTime;
	dbusResTimeErr = dbusResTime - dbusResTimeLast;
	dbusResTimeLast = dbusResTime;
	//�� ������ֵת����ͨ��ֵ
	if(SBUS_buf[0] == 0X0F && SBUS_buf[24] == 0X00)
	{
		//�Ҳ�����
		dbusCH[0] = ((int16_t)SBUS_buf[1] >> 0 | ((int16_t)SBUS_buf[2] << 8 )) & 0x07FF;
		//�Ҳ�����
		dbusCH[1] = ((int16_t)SBUS_buf[2] >> 3 | ((int16_t)SBUS_buf[3] << 5 )) & 0x07FF;
		//�������
		dbusCH[2] = ((int16_t)SBUS_buf[3] >> 6 | ((int16_t)SBUS_buf[4] << 2 ) | (int16_t)SBUS_buf[5] << 10 ) & 0x07FF;
		//�������
		dbusCH[3] = ((int16_t)SBUS_buf[5] >> 1 | ((int16_t)SBUS_buf[6] << 7 )) & 0x07FF;
		//����ť
		dbusCH[4] = ((int16_t)SBUS_buf[6] >> 4 | ((int16_t)SBUS_buf[7] << 4 )) & 0x07FF;
		//����ť
		dbusCH[5] = ((int16_t)SBUS_buf[7] >> 7 | ((int16_t)SBUS_buf[8] << 1 ) | (int16_t)SBUS_buf[9] << 9 ) & 0x07FF;
		//SWA
		dbusCH[6] = ((int16_t)SBUS_buf[9] >> 2 | ((int16_t)SBUS_buf[10] << 6 )) & 0x07FF;
		//SWB
		dbusCH[7] = ((int16_t)SBUS_buf[10] >> 5 | ((int16_t)SBUS_buf[11] << 3 )) & 0x07FF;
		//SWC
		dbusCH[8] = ((int16_t)SBUS_buf[12] << 0 | ((int16_t)SBUS_buf[13] << 8 )) & 0x07FF;
		//SWD
		dbusCH[9] = ((int16_t)SBUS_buf[13] >> 3 | ((int16_t)SBUS_buf[14] << 5 )) & 0x07FF;
		//��
		dbusCH[10] = ((int16_t)SBUS_buf[14] >> 6 | ((int16_t)SBUS_buf[15] << 2 ) | (int16_t)SBUS_buf[16] << 10 ) & 0x07FF;
		dbusCH[11] = ((int16_t)SBUS_buf[16] >> 1 | ((int16_t)SBUS_buf[17] << 7 )) & 0x07FF;
		dbusCH[12] = ((int16_t)SBUS_buf[17] >> 4 | ((int16_t)SBUS_buf[18] << 4 )) & 0x07FF;
		dbusCH[13] = ((int16_t)SBUS_buf[18] >> 7 | ((int16_t)SBUS_buf[19] << 1 ) | (int16_t)SBUS_buf[20] << 9 ) & 0x07FF;
		dbusCH[14] = ((int16_t)SBUS_buf[20] >> 2 | ((int16_t)SBUS_buf[21] << 6 )) & 0x07FF;
		dbusCH[15] = ((int16_t)SBUS_buf[21] >> 5 | ((int16_t)SBUS_buf[22] << 3 )) & 0x07FF;
		
		for(int i=0;i<10;i++)
			dbusCH[i] -= 1023;
		//������ 784
		for( int i=0;i<4;i++)
		{
			
			rc.ch[i] = dbusCH[i]/784.0f;
			if(dbusCH[6+i] == 0)
				rc.sw[i] = 0+1;
			else if(dbusCH[6+i] > 0 )
				rc.sw[i] = 1+1;
			else if(dbusCH[6+i] < 0 )
				rc.sw[i] = -1+1;
				
		}
		rc.dial[0] = dbusCH[4]/784.0f;
		rc.dial[1] = dbusCH[5]/784.0f;
		
	}
	
}


//���ݴ���
void dbusHandleData(volatile const uint8_t *SBUS_buf)
{
	dbusResTime = sysTickTime;
	dbusResTimeErr = dbusResTime - dbusResTimeLast;
	dbusResTimeLast = dbusResTime;
	//�� ������ֵת����ͨ��ֵ
	
	// ��ҡ�˺���  ��Χ+-660    [0]8bits+[1]3bits
	dbusCH[0] =((SBUS_buf[0] | (SBUS_buf[1] << 8)) & 0x07ff) - 1024; // Channel 0
	// ��ҡ������   ��Χ+-660   [1]5bits+[2]6bits
	dbusCH[1] = (((SBUS_buf[1] >> 3) | (SBUS_buf[2] << 5)) & 0x07ff) - 1024; // Channel 1
	// ��ҡ������   ��Χ+-660   [4]7bits+[5]4bits
	dbusCH[2] = (((SBUS_buf[4] >> 1) | (SBUS_buf[5] << 7)) & 0x07ff) - 1024; // Channel 3
	// ��ҡ�˺���   ��Χ+-660   [2]2bits+[3]8bits+[4]1bit
	dbusCH[3] = (((SBUS_buf[2] >> 6) | (SBUS_buf[3] << 2) | (SBUS_buf[4] << 10)) & 0x07ff) - 1024; // Channel 2
	// ���Ͻǲ��� ��Χ+-660 [16]8bis+[17]3bits
	dbusCH[4] = -(((SBUS_buf[16] | (SBUS_buf[17] << 8)) & 0x07ff) - 1024);
	// ��߿���  132 ������     [5]2bits
	dbusCH[5] = ((SBUS_buf[5] >> 4) & 0x000C) >> 2; // Switch left
	// �ұ߿���  132 ������     [5]2bits
	dbusCH[6] = ((SBUS_buf[5] >> 4) & 0x0003); // Switch right
	
	//������ 784
	for( int i=0;i<4;i++)
		rc.ch[i] = dbusCH[i]/660.0f;
		
	if(dbusCH[5] == 1)
		rc.sw[0] = 0;
	else if(dbusCH[5] == 3 )
		rc.sw[0] = 1;
	else if(dbusCH[5] == 2)
		rc.sw[0] = 2;
	
	if(dbusCH[6] == 1)
		rc.sw[3] = 0;
	else if(dbusCH[6] == 3 )
		rc.sw[3] = 1;
	else if(dbusCH[6] == 2)
		rc.sw[3] = 2;
	
	
	rc.dial[0] = dbusCH[4]/784.0f;
	rc.dial[1] = dbusCH[5]/784.0f;

	
}