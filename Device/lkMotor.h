#ifndef __LKMOTOR_H__
#define __LKMOTOR_H__


#include "main.h"
#include "board.h"
extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;


enum LKMotorCanNum : uint8_t
{
	lkCan1 = 0,
	lkCan2,
	lkCan3
};

enum LkMotorCmd : uint8_t
{
	cmdReadPid = 0x30,	//读取PID参数
	cmdWritePidRam = 0x31,//写入PID参数到RAM
	cmdWritePidRom = 0x32,//写入PID参数到ROM
	cmdCtrlTorque = 0xA1,//转矩闭环
	cmdCtrlSpeed = 0xA2,//速度闭环
	cmdCtrlPostion1 = 0xA3,//位置闭环
	
};

class CanMsg
{
	public:
	CanMsg(LKMotorCanNum _canx = lkCan1,uint16_t _canId = 0)
	{
		canx = _canx;
		canId = _canId;
		memset(data,0,8);
	}
	
	LKMotorCanNum canx;
	uint16_t canId;
	uint8_t data[8];
};


	
struct LKMotor
{
	public:
		
	//基础信息
	struct {
		uint8_t motorId;
		LKMotorCanNum canx;
		uint16_t canId;
		int32_t maxSpeedLimit;
	}info;
	
	//PID参数
	struct{
		uint8_t angKp;
		uint8_t angKi;
		uint8_t spdKp;
		uint8_t spdKi;
		uint8_t iqKp;
		uint8_t iqKi;
		uint8_t writeToRom;
	}pidSet,pidFb;
	//原始数据
	struct 
	{
		uint8_t temp;	//电机温度
		int16_t torque;	//电机力矩
		int16_t speed;	//电机速度
		int16_t encoder;	//电机编码器
		int round;
		int offsetPos;
		int16_t lastEncoder;
	}ori;
	//反馈数据
	struct
	{
		float temp;	//温度
		float torque;	//力矩
		float speed;	//速度
		float encoder;	//编码器
		float angle;	//总角度
	}fb;


	LKMotor(LKMotorCanNum canx,int motorid);	
	void readPid();
	void fbDataHandle(uint8_t *data);
	void updatePid();
	
	void ctrlCurent(float torque);
	void ctrlSpeed(float speed);
	void ctrlPositon(double pos);
	
	
};
#ifdef __cplusplus
extern "C" {
#endif

void canInit();
	
void sendMotorMsg(CanMsg &canMsg);
	
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);
	
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs);

#ifdef __cplusplus
}
#endif



#endif