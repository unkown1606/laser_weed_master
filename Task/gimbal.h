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

	//云台脱力上力
	uint8_t gimbalExhaustion();
	uint8_t gimbalOn();
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