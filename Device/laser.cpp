#include "laser.h"

#define LASER_TIM TIM3


//激光控制IO
//CustomGPIO laserIO[4] = {	CustomGPIO(GPIOD,LL_GPIO_PIN_8),
//							CustomGPIO(GPIOE,LL_GPIO_PIN_15),
//							CustomGPIO(GPIOE,LL_GPIO_PIN_14),
//							CustomGPIO(GPIOF,LL_GPIO_PIN_14)};
//				



//激光控制：包括CO2激光和mini小激光
Laser::Laser(GPIO_TypeDef* _port,uint32_t _pin,TIM_TypeDef *_timer, uint32_t _channel)
{
	port = _port;
	pin = _pin;
	timer = _timer;
	channel = _channel;
	
}

//开启激光：功率系数（0-1）
void Laser::open(float powerIndex)
{
	powerIndex =LIMIT(powerIndex,0,1);
	uint32_t ccr = powerIndex * timer->ARR;
	//调整占空比
	if(channel == LL_TIM_CHANNEL_CH1)
		LL_TIM_OC_SetCompareCH1(timer,ccr);
	else if(channel == LL_TIM_CHANNEL_CH2)
		LL_TIM_OC_SetCompareCH2(timer,ccr);
	else if(channel == LL_TIM_CHANNEL_CH3)
		LL_TIM_OC_SetCompareCH3(timer,ccr);
	else if(channel == LL_TIM_CHANNEL_CH4)
		LL_TIM_OC_SetCompareCH4(timer,ccr);
}

void Laser::close()
{
	uint32_t ccr = 0;
	//调整占空比
	if(channel == LL_TIM_CHANNEL_CH1)
		LL_TIM_OC_SetCompareCH1(timer,ccr);
	else if(channel == LL_TIM_CHANNEL_CH2)
		LL_TIM_OC_SetCompareCH2(timer,ccr);
	else if(channel == LL_TIM_CHANNEL_CH3)
		LL_TIM_OC_SetCompareCH3(timer,ccr);
	else if(channel == LL_TIM_CHANNEL_CH4)
		LL_TIM_OC_SetCompareCH4(timer,ccr);
}









//激光初始化
void laserTimInit()
{
	
    LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH3);
    //LL_TIM_EnableDMAReq_CC3(TIM2);
    LL_TIM_EnableCounter(TIM2);    // 使能计数器
    LL_TIM_EnableAllOutputs(TIM2); // 使能输出
	
	//初始化频率为5KHz
}