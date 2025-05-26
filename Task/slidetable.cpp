
#include "laser.h"

#include "lkmotor.h"
#include "dbus.h"
#include "slidetable.h"

#define SYNC_WHEEL_D 12 // 同步轮直径 mm

void TravelSwitchCallback(uint8_t id)
{
}


// 构造器:移动滑台
SlideTable::SlideTable(LKMotor *_motorX, LKMotor *_motorY, Laser *_laser, SlideTable *_previous, SlideTable *_next, int _basePriority)
	: laser(_laser), motorX(_motorX), motorY(_motorY), previous(_previous), next(_next)
{
	info.basePriority = _basePriority;
	offset.maxWaitCnt = 200;
	offset.waitSpdThreshold = 10;
	info.spdLimit.x = 10;
	info.spdLimit.y = 10;
	isLimitTable = false;
}


// 构造器2:固定滑台
SlideTable::SlideTable(float xpos)
{
	isLimitTable = false;
	pos.now.x = xpos;
}



// 滑台位置控制
void SlideTable::posCtrl(float setX, float setY)
{
	if (offset.status > 0)
		posOffsetMain();

	motorX->ctrlPositon(setX + offset.result.x);
	motorY->ctrlPositon(setY + offset.result.y);
}



// 获取滑台位置,电机一圈360,对应同步轮周长:Pi*D = 3.14159* 12mm = 37.699
Point2D SlideTable::getPos()
{
	// 角度转滑台位置 单位:mm
	pos.now.x = motorX->fb.angle * 37.69 / 360;
	pos.now.y = motorY->fb.angle * 37.69 / 360;
	return pos.now;
}


// 开启滑台位置初始化
void SlideTable::offsetStart()
{
	offset.status = 1;
}


// 滑台校准 主任务
void SlideTable::posOffsetMain()
{
	switch (offset.status)
	{
	case 0:

		break;
	case 1: // 参数初始化
		offset.xOk = offset.maxWaitCnt;
		offset.yOk = offset.maxWaitCnt;
		offset.result.clear();
		offset.status++;
		break;

	case 2:
		// 设定值自增
		if (offset.xOk > 0)
		{
			pos.set.x = pos.now.x + info.spdLimit.x;
			// 检测速度
			if (motorX->fb.speed < offset.waitSpdThreshold)
				offset.xOk--;
			else
				offset.xOk = offset.maxWaitCnt;
		}
		// 设定值自增
		if (offset.yOk > 0)
		{
			pos.set.y = pos.now.y + info.spdLimit.y;
			// 检测速度
			if (motorY->fb.speed < offset.waitSpdThreshold)
				offset.yOk--;
			else
				offset.yOk = offset.maxWaitCnt;
		}

		// 校准结束
		if (offset.xOk <= 0 && offset.yOk <= 0)
		{
			offset.status = 0;
			pos.set.x = 0;
			pos.set.y = 0;
			offset.result = pos.now;
		}
		break;
	default:
		break;
	}
}