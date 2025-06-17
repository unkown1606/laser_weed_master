#include "chassis.h"
#include <math.h>
#include "dbus.h"
#include "lkMotor.h"

// 底盘电机序号示意
//	   		wheel[1]	wheel[0]
//				 |	  		|
//				 |	  		|
//		rudder[1]|----------|rudder[0]
//				 |	  		|
//				 |	  		|
//	   		wheel[3]	 wheel[2]

//轮电机
LKMotor wheel[4] = {
	LKMotor(lkCan1,1),
	LKMotor(lkCan1,4),
	LKMotor(lkCan1,2),
	LKMotor(lkCan1,5)
};

//舵电机
LKMotor rudder[2] = {
	LKMotor(lkCan1,3),
	LKMotor(lkCan1,6)
};

//丝杠电机
LKMotor screw = LKMotor(lkCan2,3);

#define MAX_WHEEL_SPD 1200

#define RADIAN_TO_DEGREE 57.3f
#define DEGREE_TO_RADIAN 0.017452f

uint8_t enableMotorOut = 1 ;

Chassis::Chassis()
{
	mech.width = 0.875;
	mech.length = 1.045;

	// 设置PID参数
	for(uint8_t i = 0;i < 4;i ++)
	{
		wheel[i].pidSet.angKp = 50;
		wheel[i].pidSet.spdKp = 10;
		wheel[i].pidSet.curKp = 20;
		wheel[i].updatePid();
	}
	for(uint8_t i = 0;i < 2;i ++)
	{
		rudder[i].pidSet.angKp = 100;
		rudder[i].pidSet.spdKp = 80;
		rudder[i].pidSet.curKp = 100;
		rudder[i].updatePid();
	}
	screw.pidSet.angKp = 80;
	screw.pidSet.spdKp = 150;
	screw.pidSet.curKp = 200;
	screw.updatePid();
	// 记录丝杠零点
	screwPoint = screw.fb.angle;
}

void Chassis::changeWidth()
{
	// 先将舵转向
	rudder[0].ctrlPositon(90 * 9, 500);
	rudder[1].ctrlPositon(90 * 9, 500);

	// 丝杠运动解算，丝杠减速比为9:1
	screwSpd = u / 0.005 * 360;
	screwIncrement = 360 * 9 * SIGN(screwSpd);
	if(u != 0 && screw.fb.ready)
	{
		screw.ctrlIncrement(screwIncrement, (uint32_t)(9 * ABS(screwSpd)));
	}
	else
	{
		screw.ctrlSpeed(0);
	}
	screwDis = (screw.fb.angle - screwPoint) / 360 * 0.005;
	if (ABS(screwDis) > 0.001)
	{
		mech.length = 1.045 + screwDis;
	}
	if(mech.length > 1.345)
	{
		mech.length = 1.345;
	}
	else if(mech.length < 1.045)
	{
		mech.length =1.045;
	}

	// 轮运动解算
	for(uint8_t i=0;i<4;i++)
	{
		// 计算轮子速度，速度要为丝杠的一半，因为两侧轮子同时运动
		wheelSpd[i] = (i < 2 ? 1 : -1) * u / 0.115 * 360 * 0.5;
		// 四个速度的运动方向
		int8_t velSign = (i < 2 ? 1 : -1);
		incrementPos[i] = 360 * 8 * velSign;
		if(u != 0 && wheel[i].fb.ready)
		{
			// LK电机不能用速度模式控制，用增量位置模式，通过maxSpd控制速度，这样通讯连接不上时电机不会疯转
			wheel[i].ctrlIncrement(incrementPos[i], (uint32_t)(8 * ABS(wheelSpd[i])));
		}
		else
		{
			wheel[i].ctrlSpeed(0);
		}
	}
}

void Chassis::move()
{
	if(w != 0 || v!= 0)
	{
		if(w == 0)
		{
			vel[0] = vel[1] = v;
		}
		else
		{
			//阿克曼转向，转角、速度的解算
			alpha = atan2(mech.width, 2*v/w + mech.length) * RADIAN_TO_DEGREE;
			beta = atan2(mech.width, 2*v/w - mech.length) * RADIAN_TO_DEGREE;
			r[0] = mech.width / (2 * sinf(alpha * DEGREE_TO_RADIAN));
			r[1] = mech.width / (2 * sinf(beta * DEGREE_TO_RADIAN));
			vel[0] = w * r[0];
			vel[1] = w * r[1];
		}
	}
	//将轮子在第三象限的解算结果映射到第一象限
	if (alpha > 90 && beta > 90)
	{
		alpha = alpha - 180;
		beta = beta - 180;
		vel[0] = -vel[0];
		vel[1] = -vel[1];
	}
	if(v == 0)
	{
		vel[0] = vel[1] = 0;
	}
	//轮子速度单位从m/s换算到°/s
	wheelSpd[0] = wheelSpd[2] = vel[0] / 0.115 * 360;
	wheelSpd[1] = wheelSpd[3] = vel[1] / 0.115 * 360;

	//轮电机减速比为8:1，舵电机减速比为9:1
	for(uint8_t i=0;i<4;i++)
	{
		if((vel[0] || vel[1]) != 0 && wheel[i].fb.ready)
		{
			//四个速度的运动方向
			int8_t velSign = (i % 2 == 0 ? -1 : 1) * SIGN(wheelSpd[i]);
			incrementPos[i] = 360 * 8 * velSign;
			//LK电机不能用速度模式控制，用增量位置模式，通过maxSpd控制速度，这样通讯连接不上时电机不会疯转
			wheel[i].ctrlIncrement(incrementPos[i], (uint32_t)(8 * ABS(wheelSpd[i])));
		}
		else
		{
			wheel[i].ctrlSpeed(0);
		}
	}
	rudder[0].ctrlPositon(alpha * 9, 500);
	rudder[1].ctrlPositon(beta * 9, 500);
	screw.ctrlSpeed(0);
}

void Chassis::ctrl()
{
	if(ABS(rc.leftFB) < 0.1 && ABS(rc.leftLR) < 0.1)
	{
		v = w = 0;
	}
	else
	{
		// 将y=x和y=-x围成的包含x轴的区域映射到y=±x上，防止角度波动过大
		if(ABS(rc.leftFB) <  ABS(rc.leftLR))
		{
			rc.leftFB = rc.leftFB;
			rc.leftLR = rc.leftLR;
		}
		v = rc.leftFB * 0.05;	// 底盘移动速度最大为0.05m/s
		w = (v > 0 ? -1 : 1) * rc.leftLR * 0.05;	// 角速度最大为0.05rad/s
	}
	if(ABS(rc.leftFB) < 0.1)
	{
		u = 0;
	}
	else
	{
		u = rc.leftFB * 0.005*0.5;	// 丝杠移动速度最大为0.01m/s
	}
//	if(ABS(rc.rightLR) > 0.1)
//	{
//		p = -rc.rightLR * 2000;
//	}
//	else
//	{
//		p = 0;
//	}

	// swB上拨为底盘运动，下拨为丝杠运动
	if(rc.swB == 0)
	{
		move();
	}
	else
	{
		changeWidth();
	}
}

uint8_t Chassis::chassisExhaustion()
{
	for(uint8_t i = 0;i < 4;i ++)
	{
		wheel[i].stopMotor();
	}
	for(uint8_t i = 0;i < 2;i ++)
	{
		rudder[i].stopMotor();
	}
	screw.stopMotor();
	return (wheel[0].fb.isStop && wheel[1].fb.isStop && wheel[2].fb.isStop && wheel[3].fb.isStop && 
			rudder[0].fb.isStop && rudder[1].fb.isStop && screw.fb.isStop);
}

uint8_t Chassis::chassisOn()
{
	for(uint8_t i = 0;i < 4;i ++)
	{
		wheel[i].runMotor();
	}
	for(uint8_t i = 0;i < 2;i ++)
	{
		rudder[i].runMotor();
	}
	screw.runMotor();
	return (!wheel[0].fb.isStop && !wheel[1].fb.isStop && !wheel[2].fb.isStop && !wheel[3].fb.isStop && 
			!rudder[0].fb.isStop && !rudder[1].fb.isStop && !screw.fb.isStop);
//	//调试某个电机用
//	screw.runMotor();
//	return (!screw.fb.isStop);
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



