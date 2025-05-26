#ifndef __GIMBAL_H__
#define __GIMBAL_H__

#include "main.h"
#include "board.h"



#include "slidetable.h"

// ¶à»¬Ì¨¿ØÖÆ
struct MultiTable
{
public:
	SlideTable (&table)[4];

	MultiTable();

	void posCtrl();
	void startPosOffset();
	void ctrlMain();
};



#endif