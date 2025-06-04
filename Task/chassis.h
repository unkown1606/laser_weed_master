#ifndef __CHASSIS_H__
#define __CHASSIS_H__
#include "main.h"
#include "board.h"

struct Chassis
{
	public:
	Chassis();
	float r[2];				//右、左轮转弯半径
	float vel[2];			//右、左轮前向速度
	float wheelSpd[4];		//四个轮子的速度
	float rudderAng[2];		//右左两侧舵电机角度
	float trueRudderAng[4];	//
	float trueWheelSpd[4];	//
	int8_t wheelTurn[4];	//

	struct 
	{
		float width;
		float length;
		float setWidth;
	}mech;

	void ctrl(float v,float w);		//底盘的控制

	void changeWidth(float changeSpd);	//丝杠宽度改变
	//底盘脱离上力
	void chassisExhaustion();
	void chassisOn();
};



#endif