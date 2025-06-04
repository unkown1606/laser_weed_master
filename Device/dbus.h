#ifndef __DBUS_H__
#define __DBUS_H__
#include "board.h"
#include "main.h"

#include <stdint.h>

#define SBUS_RX_BUF_NUM 16 ///< DMA数据长度
#define RC_FRAME_LENGTH 25u ///<遙控器接收数据包长度
/**************************
		模式2
	ch3			ch2
	  |			 |
ch4	——+——	ch1——+——
	  |			 |
**************************/
struct FlyskyKey
{
	union
	{
		struct
		{
			float ch[4];
			float dial[2];
			int8_t sw[4];
		};
		struct
		{
			float rightLR;
			float rightFB;
			float leftFB;
			float leftLR;
			float leftDial;
			float rightDial;	
			int8_t swA;
			int8_t swB;
			int8_t swC;
			int8_t swD;
		};
		
	};
};

extern struct FlyskyKey rc;



#ifdef __cplusplus
extern "C" {
#endif

void dbusInit();
void dbusCallBack();

	
	
#ifdef __cplusplus
}
#endif


#endif