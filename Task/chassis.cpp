
#include "chassis.h"
#include <math.h>
#include "dbus.h"

#include "lkMotor.h"
//轮子电机
LKMotor wheel[4] = {
	LKMotor(lkCan1,1),
	LKMotor(lkCan1,2),
	LKMotor(lkCan1,3),
	LKMotor(lkCan1,4)
};

//舵电机
LKMotor rudder[4] = {
	LKMotor(lkCan1,5),
	LKMotor(lkCan1,6),
	LKMotor(lkCan1,7),
	LKMotor(lkCan1,8)
};

//伸缩电机
LKMotor expand[2] = {
	LKMotor(lkCan1,9),
	LKMotor(lkCan1,10)
};


#define MAX_WHEEL_SPD 1200



Chassis::Chassis()
{
	mech.width = 1.2;
	mech.length = 1.4;
}


void changeWidth(float changeSpd)
{
	
}


#define RADIAN_TO_DEGREE 57.3f
#define DEGREE_TO_RADIAN 0.017452f



uint8_t enableMotorOut =0 ;
void Chassis::ctrl(float spdX,float spdY,float spdZ)
{
	
	spdX = rc.leftFB * 2000;
	spdY = rc.leftLR * 2000;
	spdZ = rc.rightLR * 1000;
	//获取当前机器人四个轮子的间距
	float dx = 1000;
	float dy = 1000;
	float theta = atan2(dx,dy);
	float rotSinTheta = spdZ*sin(theta);
	float rotCosTheta = spdZ*cos(theta);
	
	
	wheelVelX[0] = spdX - rotSinTheta;
	wheelVelX[1] = spdX + rotSinTheta;
	wheelVelX[2] = spdX + rotSinTheta;
	wheelVelX[3] = spdX - rotSinTheta;
	
	wheelVelY[0] = spdY - rotCosTheta;
	wheelVelY[1] = spdY - rotCosTheta;
	wheelVelY[2] = spdY + rotCosTheta;
	wheelVelY[3] = spdY + rotCosTheta;
	
	for(int i=0;i<4;i++)
	{
		//求解的原始舵方向
		rudderAng[i] = atan2f(wheelVelY[i],wheelVelX[i]) * RADIAN_TO_DEGREE;
		
		wheelTurn[i] = 1;
		if(rudderAng[i] > 120)
		{
			trueRudderAng[i] = rudderAng[i] - 180;
			wheelTurn[i] = -1;
		}
		else if(rudderAng[i] < -120)
		{
			trueRudderAng[i] = rudderAng[i] + 180;
			wheelTurn[i] = -1;
		}
		else
			trueRudderAng[i] = rudderAng[i];
			
		//轮速度
		wheelSpd[i] = sqrt(wheelVelX[i]*wheelVelX[i]+wheelVelY[i]*wheelVelY[i]);
		wheelSpd[i]  = LIMIT(wheelSpd[i] ,-MAX_WHEEL_SPD,MAX_WHEEL_SPD);
		trueWheelSpd[i] = wheelSpd[i] * wheelTurn[i];
		if(enableMotorOut )
		{
			
			
		//将指令发送到电机
		rudder[i].ctrlPositon(trueRudderAng[i]*9);
		wheel[i].ctrlSpeed(trueWheelSpd[i]);
		}
	}
}





