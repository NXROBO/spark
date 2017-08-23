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

#ifndef LOCKER_H
#define LOCKER_H
namespace rs
{
namespace hal
{
class Locker
{
public:
  enum LOCK_STATUS
  {
    LOCK_OK = 1,
    LOCK_TIMEOUT = -1,
    LOCK_FAILED = 0
  };

  Locker()
  {
    init();
  }

  ~Locker()
  {
    release();
  }

  Locker::LOCK_STATUS lock(unsigned long timeout = 0xFFFFFFFF)
  {
    if (timeout == 0xFFFFFFFF)
    {
      if (pthread_mutex_lock(&_lock) == 0)
        return LOCK_OK;
    }
    else if (timeout == 0)
    {
      if (pthread_mutex_trylock(&_lock) == 0)
        return LOCK_OK;
    }
    else
    {
      timespec wait_time;
      timeval now;
      gettimeofday(&now, NULL);

      wait_time.tv_sec = timeout / 1000 + now.tv_sec;
      wait_time.tv_nsec = (timeout % 1000) * 1000000 + now.tv_usec * 1000;

      if (wait_time.tv_nsec >= 1000000000)
      {
        ++wait_time.tv_sec;
        wait_time.tv_nsec -= 1000000000;
      }
    }
    return LOCK_FAILED;
  }

  void unlock()
  {
    pthread_mutex_unlock(&_lock);
  }

  pthread_mutex_t *getLockHandle()
  {
    return &_lock;
  }

protected:
  void init()
  {
    pthread_mutex_init(&_lock, NULL);
  }

  void release()
  {
    unlock();  // force unlock before release

    pthread_mutex_destroy(&_lock);
  }

  pthread_mutex_t _lock;
};

class AutoLocker
{
public:
  AutoLocker(Locker &l) : _binded(l)
  {
    _binded.lock();
  }

  void forceUnlock()
  {
    _binded.unlock();
  }
  ~AutoLocker()
  {
    _binded.unlock();
  }
  Locker &_binded;
};
}
}
#endif
