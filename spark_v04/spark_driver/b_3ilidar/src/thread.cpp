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

#include "sdkcommon.h"
#include "thread.h"
#include "arch_linux.h"

#include <sched.h>

namespace rs
{
namespace hal
{
Thread Thread::create(thread_proc_t proc, void *data)
{
  Thread newborn(proc, data);

  // tricky code, we assume pthread_t is not a structure but a word size value
  assert(sizeof(newborn._handle) >= sizeof(pthread_t));

  pthread_create((pthread_t *)&newborn._handle, NULL, (void *(*)(void *))proc, data);

  return newborn;
}

u_result Thread::terminate()
{
  if (!this->_handle)
    return RESULT_OK;
  pthread_exit(0);
  return RESULT_OK;
}

u_result Thread::setPriority(priority_val_t p)
{
  if (!this->_handle)
    return RESULT_OPERATION_FAIL;

  // check whether current schedule policy supports priority levels

  int current_policy;
  struct sched_param current_param;
  int ans;
  if (pthread_getschedparam((pthread_t) this->_handle, &current_policy, &current_param))
  {
    // cannot retreieve values
    return RESULT_OPERATION_FAIL;
  }

  // int pthread_priority = 0 ;

  switch (p)
  {
    case PRIORITY_REALTIME:
      // pthread_priority = pthread_priority_max;
      current_policy = SCHED_RR;
      break;
    case PRIORITY_HIGH:
      // pthread_priority = (pthread_priority_max + pthread_priority_min)/2;
      current_policy = SCHED_RR;
      break;
    case PRIORITY_NORMAL:
    case PRIORITY_LOW:
    case PRIORITY_IDLE:
      // pthread_priority = 0;
      current_policy = SCHED_OTHER;
      break;
  }

  current_param.sched_priority = current_policy;
  if ((ans = pthread_setschedparam((pthread_t) this->_handle, current_policy, &current_param)))
  {
    return RESULT_OPERATION_FAIL;
  }
  return RESULT_OK;
}

Thread::priority_val_t Thread::getPriority()
{
  if (!this->_handle)
    return PRIORITY_NORMAL;

  int current_policy;
  struct sched_param current_param;
  if (pthread_getschedparam((pthread_t) this->_handle, &current_policy, &current_param))
  {
    // cannot retreieve values
    return PRIORITY_NORMAL;
  }

  int pthread_priority_max = sched_get_priority_max(SCHED_RR);
  int pthread_priority_min = sched_get_priority_min(SCHED_RR);

  if (current_param.sched_priority == (pthread_priority_max))
  {
    return PRIORITY_REALTIME;
  }
  if (current_param.sched_priority >= (pthread_priority_max + pthread_priority_min) / 2)
  {
    return PRIORITY_HIGH;
  }
  return PRIORITY_NORMAL;
}

u_result Thread::join(unsigned long timeout)
{
  if (!this->_handle)
    return RESULT_OK;

  pthread_join((pthread_t)(this->_handle), NULL);
  return RESULT_OK;
}
}
}
