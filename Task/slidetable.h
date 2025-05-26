#ifndef __SLIDETABLE_H__
#define __SLIDETABLE_H__
#include "main.h"
#include "board.h"

#include "laser.h"
#include "lkmotor.h"

enum TableStatus : int
{
	tableFinding = 0,	 // 滑台寻找中
	tableAiming = 10,	 // 滑台瞄准中
	tableEmitting = 100, // 滑台发射中
};

struct Point2D
{
public:
	float x;
	float y;
	Point2D(float _x = 0, float _y = 0)
	{
		x = _x;
		y = _y;
	}
	Point2D operator=(const Point2D &next)
	{
		Point2D result;
		result.x = next.x;
		result.y = next.y;
		return result;
	}

	Point2D operator+(const Point2D &next)
	{
		Point2D result;
		result.x = this->x + next.x;
		result.y = this->y + next.y;
		return result;
	}

	void clear()
	{
		x = 0;
		y = 0;
	}
};

// 单个滑台
class SlideTable
{
public:
	Laser *laser; // 激光控制接口
	LKMotor *motorX;
	LKMotor *motorY;

	struct
	{
		Point2D now;		// 当前位置
		Point2D set;		// 当前位置
		Point2D defaultSet; // 默认设定值
		Point2D pushSet;	// 推挤设定值
		Point2D resultSet;	// 设定值结果
		Point2D limitUp;		//运动空间限制
		Point2D limitDown;		//运动空间限制
	} pos;

	struct
	{
		Point2D spdLimit;
		int basePriority;
		bool xCollide; // 碰撞
		bool yCollide; // 碰撞
	} info;

	struct
	{
		Point2D result;
		uint8_t status;
		int xOk;
		int yOk;
		int maxWaitCnt;
		float waitSpdThreshold;
	} offset;

	// 优先级
	int priority;
	bool updateFalg;
	TableStatus status;
	SlideTable *previous;
	SlideTable *next;
	bool isLimitTable;

	SlideTable(LKMotor *_motorX, LKMotor *_motorY, Laser *_laser, SlideTable *_previous, SlideTable *_next, int _basePriority);
	SlideTable(float xpos);

	void posCtrl(float setX, float setY);
	void posOffsetMain();
	void offsetStart();
	void speedLimit(float xLimit, float yLimit);
	Point2D getPos();
};

#endif