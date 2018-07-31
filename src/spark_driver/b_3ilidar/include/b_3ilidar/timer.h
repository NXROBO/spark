/*
*  RSLIDAR System
*  Driver Interface
*
*  Copyright 2015 RS Team
*  All rights reserved.
*
*	Author: ruishi, Data:2015-12-25
*
*/

#ifndef TIMER_H
#define TIMER_H

#include "rstypes.h"

// TODO: the highest timer interface should be clock_gettime
namespace rs
{
namespace arch
{
_u64 rs_getus();
_u32 rs_getms();
}
}

#define getms() rs::arch::rs_getms()
#endif
