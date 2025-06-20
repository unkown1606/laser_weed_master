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
	float yawPos;	// yaw位置
	float pitchSpd;		// pitch速度
	float pitchPos;	// pitch位置

	//云台脱力上力
	uint8_t gimbalExhaustion();
	uint8_t gimbalOn();

	void runMotor();	// 云台电机正常运行
	float laserPWM;// 激光PWM值

	// 开启激光
	void runLaser();
};

enum GimbalLmimitPos : uint16_t
{
	limitYawHigh = 280,	// yaw电机运动范围最大值
	limitYawLow = 10,	// yaw电机运动范围最小值
	limitPitchHigh = 160,	// pitch电机运动范围最大值
	limitPitchLow = 0,	// pitch电机运动范围最小值
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