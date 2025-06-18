#ifndef __GIMBAL_H__
#define __GIMBAL_H__

#include "main.h"
#include "board.h"

//云台
struct Gimbal
{
public:
	Gimbal();
	void ctrlMain();

	float x;	//yaw电机控制值
	float y;	//pitch电机控制值
	float yawSpd;		// yaw速度
	float yawIncrement;	// yaw位置增量
	float pitchSpd;		// pitch速度
	float pitchIncrement;	// pitch位置增量

	//云台脱力上力
	uint8_t gimbalExhaustion();
	uint8_t gimbalOn();

	float laserPWM;// 激光PWM值

	// 开启激光
	void runLaser();
};


// 多滑台控制
//struct MultiTable
//{
//public:
//	SlideTable (&table)[4];

//	MultiTable();

//	void posCtrl();
//	void startPosOffset();
//	void ctrlMain();
//};



#endif