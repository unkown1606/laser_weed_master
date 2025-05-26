#ifndef __LASER_H__
#define __LASER_H__
#include "main.h"
#include "board.h"


//激光控制：包括CO2激光和mini小激光
struct Laser
{
public:
	GPIO_TypeDef* port;
	uint32_t pin;
	TIM_TypeDef *timer;
	uint32_t channel;
	Laser(GPIO_TypeDef* _port,uint32_t _pin,TIM_TypeDef *_timer, uint32_t _channel);
	
	//开启激光：功率系数（0-1）
	void open(float powerIndex);
	
	void close();

};



#ifdef __cplusplus
extern "C" {
#endif

void laserTimInit();

#ifdef __cplusplus
}
#endif


#endif