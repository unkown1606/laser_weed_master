#include "chassis.h"
#include <math.h>
#include "dbus.h"
#include "lkMotor.h"

//轮电机
LKMotor wheel[4] = {
	LKMotor(lkCan1,1),
	LKMotor(lkCan1,2),
	LKMotor(lkCan1,3),
	LKMotor(lkCan1,4)
};

//舵电机
LKMotor rudder[2] = {
	LKMotor(lkCan1,6),
	LKMotor(lkCan1,5)
};

//丝杠电机
LKMotor screw = LKMotor(lkCan1,7);

#define MAX_WHEEL_SPD 1200

#define RADIAN_TO_DEGREE 57.3f
#define DEGREE_TO_RADIAN 0.017452f

uint8_t enableMotorOut = 1 ;

//LK电机不能用速度模式控制，应该用位置模式，当通讯连接不上时电机会持续执行最后的命令

Chassis::Chassis()
{
	mech.width = 1.2;
	mech.length = 1.4;

	// 设置PID参数
	for(uint8_t i = 0;i < 4;i ++)
	{
		wheel[i].pidSet.angKp = 1;
		wheel[i].pidSet.spdKp = 1;
		wheel[i].pidSet.iqKp = 1;
		wheel[i].updatePid();
	}
	for(uint8_t i = 0;i < 2;i ++)
	{
		rudder[i].pidSet.angKp = 1;
		rudder[i].pidSet.spdKp = 1;
		rudder[i].pidSet.iqKp = 1;
		rudder[i].updatePid();
	}
	screw.pidSet.angKp = 1;
	screw.pidSet.spdKp = 1;
	screw.pidSet.iqKi = 1;
	screw.updatePid();
}

void Chassis::changeWidth()
{
	screw.ctrlSpeed(0);
}

void Chassis::move()
{
	if(isTurning == 1)
	{
	alpha = atan2(mech.width, 2*v/w + mech.length) * RADIAN_TO_DEGREE;
	beta = atan2(mech.width, 2*v/w - mech.length) * RADIAN_TO_DEGREE;
	r[0] = mech.width / (2 * sinf(alpha * DEGREE_TO_RADIAN));
	r[1] = mech.width / (2 * sinf(beta * DEGREE_TO_RADIAN));
	}
	vel[0] = w * r[0];
	vel[1] = w * r[1];

	if (alpha > 90 && beta > 90)
	{
		alpha = alpha - 180;
		beta = beta - 180;
		vel[0] = -vel[0];
		vel[1] = -vel[1];
	}
	wheelSpd[0] = wheelSpd[2] = vel[0];
	wheelSpd[1] = wheelSpd[3] = vel[1];

	wheel[0].ctrlPosIncrement(0);
	wheel[1].ctrlPosIncrement(0);
	wheel[2].ctrlPosIncrement(0);
	wheel[3].ctrlPosIncrement(0);
	rudder[0].ctrlPositon(-beta * 9);
	rudder[1].ctrlPositon(-alpha * 9);
}

void Chassis::ctrl()
{
	if(ABS(rc.leftFB) > 0.1 && ABS(rc.leftLR) > 0.1)
	{
		//遥控器加偏置
		v = rc.leftFB * 2000;
		w = (v > 0 ? -1 : 1) * rc.leftLR * 2000;
		isTurning = 1;
	}
	else
	{
		v = w = 0;
		isTurning = 0;
	}
	if(ABS(rc.rightLR) > 0.1)
	{
		u = -rc.rightLR * 2000;
	}
	else
	{
		u = 0;
	}

	move();
	changeWidth();
}

void Chassis::chassisExhaustion()
{
//	wheel[0].stopMotor();
//	wheel[1].stopMotor();
//	wheel[2].stopMotor();
//	wheel[3].stopMotor();
//	if(!rudder[0].fb.isStop)
//	rudder[0].stopMotor();
//	if(!rudder[1].fb.isStop)
//	rudder[1].stopMotor();
//	if(!screw.fb.isStop)
//	screw.stopMotor();
}

void Chassis::chassisOn()
{
//	for(uint8_t i = 0;i < 4;i ++)
//	{
//		wheel[i].runMotor();
//	}
//	for(uint8_t i = 0;i < 2;i ++)
//	{
//		rudder[i].runMotor();
//	}
//	screw.runMotor();
}

	//获取当前机器人四个轮子的间距
//	float dx = 1000;
//	float dy = 1000;
//	float theta = atan2(dx,dy);
//	float rotSinTheta = spdZ*sin(theta);
//	float rotCosTheta = spdZ*cos(theta);
//	
//	
//	wheelVelX[0] = spdX - rotSinTheta;
//	wheelVelX[1] = spdX + rotSinTheta;
//	wheelVelX[2] = spdX + rotSinTheta;
//	wheelVelX[3] = spdX - rotSinTheta;
//	
//	wheelVelY[0] = spdY - rotCosTheta;
//	wheelVelY[1] = spdY - rotCosTheta;
//	wheelVelY[2] = spdY + rotCosTheta;
//	wheelVelY[3] = spdY + rotCosTheta;
//	
//	for(int i=0;i<2;i++)
//	{
//		//求解的原始舵方向
//		rudderAng[i] = atan2f(wheelVelY[i],wheelVelX[i]) * RADIAN_TO_DEGREE;
//		
//		wheelTurn[i] = 1;
//		if(rudderAng[i] > 120)
//		{
//			trueRudderAng[i] = rudderAng[i] - 180;
//			wheelTurn[i] = -1;
//		}
//		else if(rudderAng[i] < -120)
//		{
//			trueRudderAng[i] = rudderAng[i] + 180;
//			wheelTurn[i] = -1;
//		}
//		else
//			trueRudderAng[i] = rudderAng[i];
//			
//		//轮速度
//		wheelSpd[i] = sqrt(wheelVelX[i]*wheelVelX[i]+wheelVelY[i]*wheelVelY[i]);
//		wheelSpd[i]  = LIMIT(wheelSpd[i] ,-MAX_WHEEL_SPD,MAX_WHEEL_SPD);
//		trueWheelSpd[i] = wheelSpd[i] * wheelTurn[i];
//	}
//	if(enableMotorOut )
//	{
//		//将指令发送到电机
//		for(int i=0;i<2;i++)
//		{
//			rudder[i].ctrlPositon(trueRudderAng[i]*9);
//		}
//		for(int i=0;i<4;i++)
//		{
//			wheel[i].ctrlSpeed(trueWheelSpd[i]);
//		}
//	}



