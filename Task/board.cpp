#include "board.h"
#include "main.h"
#include "dbus.h"
#include "laser.h"
#include "lkMotor.h"
#include "vision.h"
#include "chassis.h"
#include "gimbal.h"
Chassis chassis;
//MultiTable multiTable;
Gimbal gimbal;

#define TICK_PER_SECOND 1000
#define TICK_US (1000000 / TICK_PER_SECOND)

//-Wno-deprecated-register
uint64_t sysTickTime =0;

void allInit()
{
	
    SysTick_Config(SystemCoreClock / TICK_PER_SECOND);
	
    // 滴答定时器优先级初始化
    //NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 4, 0));
	//遥控器初始化
	dbusInit();
	//laserTimInit();
	
	visionInit();
	canInit();
	
	
	
}

//2O 3R 4G
void schedule()
{
	sysTickTime++;
	
	if(sysTickTime % 500 ==0)
	{
		LL_GPIO_TogglePin(GPIOG,LL_GPIO_PIN_7);
		LL_GPIO_TogglePin(GPIOF,LL_GPIO_PIN_4);
	}
	
	
	if(sysTickTime%2 == 0)
	{
		chassis.ctrl(0,0,0);
//		multiTable.ctrlMain();
		gimbal.ctrlMain();
	}
	
	
}







