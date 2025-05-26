#ifndef __VISION_H__
#define __VISION_H__

#include "main.h"
#include "board.h"

struct NUCToBoard{
	uint8_t header;
	float xPos[4];
	float yPos[4];
	
	uint16_t crc16;
};


struct BoardToNUC{
	uint8_t header;
	
	
	uint16_t crc16;
	
	
};

	


#ifdef __cplusplus
extern "C" {
#endif

void visionInit();
void visionCallBack();
	
void visionSend();
	
#ifdef __cplusplus
}
#endif


#endif