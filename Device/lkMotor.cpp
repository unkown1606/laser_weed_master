#include "lkMotor.h"
#include <map>
// 通过can线和ID映射电机对象
LKMotor* lkMotorMap[3][8];

void sendMotorMsg(CanMsg &canMsg);

// 构造器
LKMotor::LKMotor(LKMotorCanNum canx, int motorid)
{
	info.motorId = motorid;
	info.canId = motorid + 0x140;
	info.canx = canx;

	// 添加映射
	lkMotorMap[canx][info.motorId-1] = this;
	pidSet.writeToRom = 0;
	ori.lastEncoder = 8191;
	info.maxSpeedLimit = 5000;	//dps
}


// 读取PID参数
void LKMotor::readPid()
{
	CanMsg msg(info.canx, info.canId);
	msg.data[0] = cmdReadPid;

	sendMotorMsg(msg);
}


// 更新PID参数，通过Debug界面修改数据后自动更新
void LKMotor::updatePid()
{
	bool isPidNoChange = true;
	isPidNoChange = pidSet.angKp == pidFb.angKp && pidSet.spdKp == pidFb.spdKp && pidSet.iqKp == pidFb.iqKp &&
					pidSet.angKi == pidFb.angKi && pidSet.spdKi == pidFb.spdKi && pidSet.iqKi == pidFb.iqKi;
	// 如果存在参数改变
	if (!isPidNoChange)
	{
		CanMsg msg(info.canx, info.canId);
		msg.data[0] = cmdWritePidRam;
		msg.data[2] = pidSet.angKp;
		msg.data[3] = pidSet.angKi;
		msg.data[4] = pidSet.spdKp;
		msg.data[5] = pidSet.spdKi;
		msg.data[6] = pidSet.iqKp;
		msg.data[7] = pidSet.iqKi;
		sendMotorMsg(msg);
	}

	// 更新数据到电机ROM
	if (pidSet.writeToRom != 0)
	{
		CanMsg msg(info.canx, info.canId);
		msg.data[0] = cmdWritePidRom;
		msg.data[2] = pidSet.angKp;
		msg.data[3] = pidSet.angKi;
		msg.data[4] = pidSet.spdKp;
		msg.data[5] = pidSet.spdKi;
		msg.data[6] = pidSet.iqKp;
		msg.data[7] = pidSet.iqKi;
		sendMotorMsg(msg);
	}
}

// 电机关闭
void LKMotor::stopMotor()
{
	CanMsg msg(info.canx, info.canId);
	msg.data[0] = cmdCtrlStop;
	sendMotorMsg(msg);
}

// 电机开启
void LKMotor::runMotor()
{
	CanMsg msg(info.canx, info.canId);
	msg.data[0] = cmdCtrlRun;
	sendMotorMsg(msg);
}

// 控制力矩：Nm
void LKMotor::ctrlCurent(float torque)
{
	CanMsg msg(info.canx, info.canId);
	msg.data[0] = cmdCtrlTorque;

	int16_t iqControl = LIMIT(torque, -2000, 2000);
	msg.data[4] = *(uint8_t *)(&iqControl);
	msg.data[5] = *((uint8_t *)(&iqControl) + 1);
	sendMotorMsg(msg);
}


// 速度控制： dps
void LKMotor::ctrlSpeed(float speed)
{
	CanMsg msg(info.canx, info.canId);
	msg.data[0] = cmdCtrlSpeed;

	int32_t spdControl = LIMIT(speed, -info.maxSpeedLimit, info.maxSpeedLimit)*100;
	msg.data[4] = *(uint8_t *)(&spdControl);
	msg.data[5] = *((uint8_t *)(&spdControl) + 1);
	msg.data[6] = *((uint8_t *)(&spdControl) + 2);
	msg.data[7] = *((uint8_t *)(&spdControl) + 3);
	sendMotorMsg(msg);
}

// 多圈位置控制
void LKMotor::ctrlPositon(double pos, uint16_t maxSpeed)
{
	CanMsg msg(info.canx, info.canId);
	msg.data[0] = cmdCtrlPostion1;

	int32_t posControl =pos*100;
	msg.data[2] = *(uint8_t *)(&maxSpeed);
	msg.data[3] = *((uint8_t *)(&maxSpeed) + 1);
	msg.data[4] = *(uint8_t *)(&posControl);
	msg.data[5] = *((uint8_t *)(&posControl) + 1);
	msg.data[6] = *((uint8_t *)(&posControl) + 2);
	msg.data[7] = *((uint8_t *)(&posControl) + 3);
	sendMotorMsg(msg);
}

// 位置增量控制
void LKMotor::ctrlIncrement(double pos, uint32_t maxSpeed)
{
	CanMsg msg(info.canx, info.canId);
	msg.data[0] = cmdCtrlIncrement;

	int32_t angleIncrement =pos*100;
	msg.data[2] = *(uint8_t *)(&maxSpeed);
	msg.data[3] = *((uint8_t *)(&maxSpeed) + 1);
	msg.data[4] = *(uint8_t *)(&angleIncrement);
	msg.data[5] = *((uint8_t *)(&angleIncrement) + 1);
	msg.data[6] = *((uint8_t *)(&angleIncrement) + 2);
	msg.data[7] = *((uint8_t *)(&angleIncrement) + 3);
	sendMotorMsg(msg);
}

// 处理CAN反馈数据
void LKMotor::fbDataHandle(uint8_t *data)
{
	uint8_t cmd = data[0];

	// 更新PID参数
	if (cmd == cmdReadPid || cmd == cmdWritePidRam || cmd == cmdWritePidRom)
	{
		pidFb.angKp = data[2];
		pidFb.angKi = data[3];
		pidFb.spdKp = data[4];
		pidFb.spdKi = data[5];
		pidFb.iqKp = data[6];
		pidFb.iqKi = data[7];
	}
	// 控制指令反馈
	else if (cmd == cmdCtrlTorque || cmd == cmdCtrlSpeed || cmd == cmdCtrlPostion1 || cmd == cmdCtrlIncrement)
	{
		// 原始数据
		ori.temp = data[1];
		ori.torque = data[2] | (data[2] << 8);
		ori.speed = data[4] | (data[5] << 8);
		ori.encoder = data[6] | (data[7] << 8);
		// 处理
		fb.temp = ori.temp;
		fb.torque = ori.torque;
		fb.speed = ori.speed;
		fb.encoder = ori.encoder * 360 / 65536.0f;
		// 过圈处理
		if (ori.encoder - ori.lastEncoder > 8192)
			ori.round--;
		else if (ori.encoder - ori.lastEncoder < -8192)
			ori.round++;
		 ori.lastEncoder = ori.encoder;
		fb.angle = ori.round * 360 + fb.encoder;
		fb.ready = 1;
	}
	else if (cmd == cmdCtrlStop || cmd == cmdCtrlRun)
	{
		fb.isStop = cmd == cmdCtrlStop;
	}
}

/******************************************************************/

void motorCallBack(LKMotorCanNum canx, uint16_t canId, uint8_t *data)
{
	int indexId = canId-0x141;
	//根据总线和ID的映射关系处理数据到对应的电机对象
	if (indexId>=0 && indexId<8 && lkMotorMap[canx][indexId] != NULL)
	{
		lkMotorMap[canx][indexId]->fbDataHandle(data);
	}
}

/**HAL CAN*************************************************************** */


FDCAN_TxHeaderTypeDef txHeader;
FDCAN_RxHeaderTypeDef rxHeader;


void canFilterInit(FDCAN_HandleTypeDef& canHandle)
{
	
	
}

void canInit()
{
	/***********************************************************************/
	FDCAN_FilterTypeDef sFilterConfig;
	/* Configure Rx filter */
    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = 0x000;
    sFilterConfig.FilterID2 = 0x7ff;
    HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig);
	
    /* Configure global filter:
       Filter all remote frames with STD and EXT ID
       Reject non matching frames with STD ID and EXT ID */
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
    /*Configure the Rx FIFO operation mode：
      Rx FIFO overwrite mode*/
    HAL_FDCAN_ConfigRxFifoOverwrite(&hfdcan1, FDCAN_RX_FIFO0, FDCAN_RX_FIFO_OVERWRITE);

    HAL_FDCAN_Start(&hfdcan1);

    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_ERROR_PASSIVE | FDCAN_IT_ERROR_WARNING | FDCAN_IT_BUS_OFF, 0);
	
	
	/***********************************************************************/
	FDCAN_FilterTypeDef sFilterConfig2;
	/* Configure Rx filter */
    sFilterConfig2.IdType = FDCAN_STANDARD_ID;
    sFilterConfig2.FilterIndex = 0;
    sFilterConfig2.FilterType = FDCAN_FILTER_RANGE;
    sFilterConfig2.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig2.FilterID1 = 0x000;
    sFilterConfig2.FilterID2 = 0x7ff;
    HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig2);
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
    HAL_FDCAN_ConfigRxFifoOverwrite(&hfdcan2, FDCAN_RX_FIFO0, FDCAN_RX_FIFO_OVERWRITE);
    HAL_FDCAN_Start(&hfdcan2);
    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_ERROR_PASSIVE | FDCAN_IT_ERROR_WARNING | FDCAN_IT_BUS_OFF, 0);
	
	
	/***********************************************************************/
	FDCAN_FilterTypeDef sFilterConfig3;
	/* Configure Rx filter */
    sFilterConfig3.IdType = FDCAN_STANDARD_ID;
    sFilterConfig3.FilterIndex = 0;
    sFilterConfig3.FilterType = FDCAN_FILTER_RANGE;
    sFilterConfig3.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig3.FilterID1 = 0x000;
    sFilterConfig3.FilterID2 = 0x7ff;
    HAL_FDCAN_ConfigFilter(&hfdcan3, &sFilterConfig3);
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
    HAL_FDCAN_ConfigRxFifoOverwrite(&hfdcan3, FDCAN_RX_FIFO0, FDCAN_RX_FIFO_OVERWRITE);
    HAL_FDCAN_Start(&hfdcan3);
    HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_ERROR_PASSIVE | FDCAN_IT_ERROR_WARNING | FDCAN_IT_BUS_OFF, 0);
	
	
	
	
	
	
	//发送帧 帧信息填充
	txHeader.Identifier = 0;
	txHeader.IdType = FDCAN_STANDARD_ID;
	txHeader.TxFrameType = FDCAN_DATA_FRAME;
	txHeader.DataLength = FDCAN_DLC_BYTES_8;
	txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	txHeader.BitRateSwitch = FDCAN_BRS_OFF;
	txHeader.FDFormat = FDCAN_CLASSIC_CAN;
	txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
}

// 总线错误回调
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
	CLEAR_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_INIT);
}

// 总线接收中断
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
	{
		/* Retrieve Rx messages from RX FIFO0 */
		if (hfdcan == &hfdcan1)
		{
			uint8_t dataTemp[8];
			HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxHeader, dataTemp);
			// 用户回调函数
			motorCallBack(lkCan1,rxHeader.Identifier, dataTemp);
		}
		else if (hfdcan == &hfdcan2)
		{
			uint8_t dataTemp[8];
			HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxHeader, dataTemp);
			// 用户回调函数
			motorCallBack(lkCan2,rxHeader.Identifier, dataTemp);
		}
		else if (hfdcan == &hfdcan3)
		{
			uint8_t dataTemp[8];
			HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxHeader, dataTemp);
			// 用户回调函数
			motorCallBack(lkCan3,rxHeader.Identifier, dataTemp);
		}
	}
}

void sendMotorMsg(CanMsg &canMsg)
{
	FDCAN_HandleTypeDef *hfdcanx;
	//选择CAN总线
	if(canMsg.canx == lkCan1)
		hfdcanx = &hfdcan1;
	else if(canMsg.canx == lkCan2)
		hfdcanx = &hfdcan2;
	else if(canMsg.canx == lkCan3)
		hfdcanx = &hfdcan3;
	
	
	//将消息添加到发送邮箱
	if (HAL_FDCAN_GetTxFifoFreeLevel(hfdcanx) >= 0)
	{
		txHeader.Identifier = canMsg.canId;
		HAL_FDCAN_AddMessageToTxFifoQ(hfdcanx, &txHeader, canMsg.data);
	}
}

