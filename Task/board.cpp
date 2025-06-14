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
uint64_t sysTickTime = 0;
uint8_t isExhaustion = 1;

void allInit()
{

    SysTick_Config(SystemCoreClock / TICK_PER_SECOND);
	
    // 滴答定时器优先级初始化
    NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 4, 0));
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
	
	if(sysTickTime % 500 == 0)
	{
		LL_GPIO_TogglePin(GPIOG,LL_GPIO_PIN_7);
		LL_GPIO_TogglePin(GPIOF,LL_GPIO_PIN_4);
	}

	if(sysTickTime % 20 == 0)
	{
		//SWB脱力上力
		if(rc.swA == 0 && isExhaustion == 1)
		{
			uint8_t chassisOK = chassis.chassisExhaustion();
			uint8_t gimbalOK = gimbal.gimbalExhaustion();
			if(chassisOK && gimbalOK)
			{
				isExhaustion = 0;
			}
		}
		else if(rc.swA != 0 && isExhaustion == 0)
		{
			uint8_t chassisOK = chassis.chassisOn();
			uint8_t gimbalOK = gimbal.gimbalOn();
			if(chassisOK && gimbalOK)
			{
				isExhaustion = 1;
			}
		}
	}

	if(sysTickTime % 2 == 0)
	{
		chassis.ctrl();
//		multiTable.ctrlMain();
		gimbal.ctrlMain();
	}
}







