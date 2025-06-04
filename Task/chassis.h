#ifndef __CHASSIS_H__
#define __CHASSIS_H__
#include "main.h"
#include "board.h"

struct Chassis
{
	public:
	Chassis();

	float v;	//前向速度v
	float w;	//转向角速度w
	float u;	//丝杠速度
	float alpha;	//右轮转向角
	float beta;		//左轮转向角

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

	void ctrl();		//底盘的控制
	void changeWidth();	//丝杠宽度改变
	void move();		//移动转向

	//底盘脱离上力
	void chassisExhaustion();
	void chassisOn();
};



#endif