#include "gimbal.h"
#include "laser.h"
#include "lkmotor.h"
#include "dbus.h"

// 尺寸（mm）
#define RAIL_WIDTH 100	 // 滑轨宽度
#define RAIL_LENGTH 200	 // 滑轨长度
#define TABLE_WIDTH 100	 // 滑台宽度
#define TABLE_LENGTH 200 // 滑台长度

Laser laser[4] = {
	Laser(GPIOD, LL_GPIO_PIN_8, TIM2, LL_TIM_CHANNEL_CH3),
	Laser(GPIOE, LL_GPIO_PIN_15, TIM2, LL_TIM_CHANNEL_CH4),
	Laser(GPIOE, LL_GPIO_PIN_14, TIM2, LL_TIM_CHANNEL_CH1),
	Laser(GPIOF, LL_GPIO_PIN_14, TIM2, LL_TIM_CHANNEL_CH2),
};

LKMotor yaw = LKMotor(lkCan2, 2);
LKMotor pitch = LKMotor(lkCan2, 3);

Gimbal::Gimbal()
{
	
};

void Gimbal::ctrlMain()
{
	yaw.ctrlSpeed(0);
	pitch.ctrlSpeed(0);
}

void Gimbal::gimbalExhaustion()
{
//	yaw.stopMotor();
//	pitch.stopMotor();
}

void Gimbal::gimbalOn()
{
//	yaw.runMotor();
//	pitch.runMotor();
}

//// X电机 前后
//LKMotor slideMotorX[4] = {
//	LKMotor(lkCan2, 1),
//	LKMotor(lkCan2, 3),
//	LKMotor(lkCan3, 1),
//	LKMotor(lkCan3, 3)};

//// Y电机 左右
//LKMotor slideMotorY[4] = {
//	LKMotor(lkCan2, 2),
//	LKMotor(lkCan2, 4),
//	LKMotor(lkCan3, 2),
//	LKMotor(lkCan3, 4)};

//// 上下限位导轨
//SlideTable upLimitTable(RAIL_LENGTH + TABLE_LENGTH);
//SlideTable dwonLimitTable(TABLE_LENGTH);

//#define TABLE_CNT 4
//// 四个滑台的结构体
//SlideTable slideTable[TABLE_CNT] = {
//	SlideTable(&slideMotorX[0], &slideMotorY[0], &laser[0], &upLimitTable, &slideTable[1], 1),
//	SlideTable(&slideMotorX[1], &slideMotorY[1], &laser[1], &slideTable[0], &slideTable[2], 2),
//	SlideTable(&slideMotorX[2], &slideMotorY[2], &laser[2], &slideTable[1], &slideTable[3], 3),
//	SlideTable(&slideMotorX[3], &slideMotorY[3], &laser[3], &slideTable[2], &dwonLimitTable, 4),
//};



//MultiTable::MultiTable() : table(slideTable)
//{
//}




//float setMotortestValue = 0;
////多滑台控制 主程序
//void MultiTable::ctrlMain()
//{
//	slideTable[0].motorY->ctrlPositon(setMotortestValue);
//	slideTable[0].motorX->ctrlPositon(setMotortestValue);
////	slideTable[2].motorX->ctrlPositon(setMotortestValue);
////	slideTable[2].motorY->ctrlPositon(setMotortestValue);
//	
//}






////多滑台位置控制
//void MultiTable::posCtrl()
//{
//	// 首先更新四个滑台的位置
//	for (int i = 0; i < TABLE_CNT; i++)
//	{
//		// 更新滑台位置
//		//table[i].getPos();
//		// 更新滑台当前前后限幅
//		table[i].pos.limitUp.x = table[i].previous->pos.now.x - TABLE_LENGTH;
//		table[i].pos.limitDown.x = table[i].next->pos.now.x + TABLE_LENGTH;
//		// 取消更新标志位
//		table[i].updateFalg = 0;

//		// 根据状态更新优先级
//		table[i].priority = table[i].info.basePriority + table[i].status;
//	}

//	while (1)
//	{
//		SlideTable *tablePosPorityMax;
//		// 查找最高优先级 并且 没有更新的滑台进行处理
//		int maxPority = 0;
//		for (int i = 0; i < TABLE_LENGTH; i++)
//		{
//			if (table[i].updateFalg == false && table[i].priority > maxPority)
//			{
//				maxPority = table[i].priority;
//				tablePosPorityMax = &table[i];
//			}
//		}
//		// 确定所有滑台的设定值
//		//  递归处理 推挤

//		// 判断是否所有滑台都更新
//		bool isAlltableUpdate = true;
//		for (int i = 0; i < TABLE_CNT; i++)
//		{
//			if (table[i].updateFalg == false)
//				isAlltableUpdate = false;
//		}
//		// 所有滑台更新完毕退出
//		if (isAlltableUpdate)
//			break;
//	}
//}




///**
// * @brief 滑台推挤递归
// * @param node 推挤的节点
// * @param pushTar 目标位置
// * @note 高优先级推挤低优先级
// * @return 实际推挤到的位置
// */
//float pushTable(SlideTable *node, float pushTar)
//{
//	// 如果该节点是上下限位节点,则无法推挤,返回限位位置
//	if (node->isLimitTable)
//		return node->pos.now.x;

//	// 判断set是否推挤、推挤方向
//	if (pushTar > node->pos.limitUp.x)
//		// 前向推挤
//		node->pos.set.x = pushTable(node->previous, pushTar + TABLE_LENGTH) - TABLE_LENGTH;
//	else if (pushTar < node->pos.limitDown.x)
//		// 后向推挤
//		node->pos.set.x = pushTable(node->next, pushTar - TABLE_LENGTH) + TABLE_LENGTH;
//	else
//		// 不需要推挤的情况下,决定位置
//		node->pos.set.x = pushTar;

//	node->updateFalg = true;
//	return node->pos.set.x;
//}
