#ifndef __CHASSIS_H__
#define __CHASSIS_H__
#include "main.h"
#include "board.h"

struct Chassis
{
	public:
	Chassis();
		//�ĸ�����ƽ�����ٶ�
	float wheelVelX[4];
	float wheelVelY[4];
	float wheelSpd[4];	//�����ٶ�
	float rudderAng[4];	//��Ƕ�
	float trueRudderAng[4];//ʵ��ʹ�õĶ淽��
	float trueWheelSpd[4];//ʵ��ʹ�õ����ٶ�
	int8_t wheelTurn[4];	//�����Ƿ���Ҫ��ת
	
	struct 
	{
		float width;	//������
		float length;	//������
		float setWidth;
	}mech;
	//���̿���
	void ctrl(float spdX,float spdY,float spdZ);
	//�ı���
	void changeWidth(float changeSpd);
	
	
};



#endif