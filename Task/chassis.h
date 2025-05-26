#ifndef __CHASSIS_H__
#define __CHASSIS_H__
#include "main.h"
#include "board.h"

struct Chassis
{
	public:
	Chassis();
		//四个轮子平移线速度
	float wheelVelX[4];
	float wheelVelY[4];
	float wheelSpd[4];	//轮子速度
	float rudderAng[4];	//舵角度
	float trueRudderAng[4];//实际使用的舵方向
	float trueWheelSpd[4];//实际使用的轮速度
	int8_t wheelTurn[4];	//轮子是否需要反转
	
	struct 
	{
		float width;	//机身宽度
		float length;	//机身长度
		float setWidth;
	}mech;
	//底盘控制
	void ctrl(float spdX,float spdY,float spdZ);
	//改变宽度
	void changeWidth(float changeSpd);
	
	
};



#endif