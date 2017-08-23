/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, NXROBO Ltd.
 *  Xiankai Chen <xiankai.chen@nxrobo.com>
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef SPARK_BASE_MYLOCK_H_
#define SPARK_BASE_MYLOCK_H_

#include <pthread.h>
#include <iostream>

using namespace std;

namespace nxsparkbase
{
// locker interface
class ILock
{
public:
  virtual ~ILock()
  {
  }

  virtual void Lock()
  {
    cout << "ILock" << endl;
  }
  virtual void Unlock()
  {
    cout << "ILock Unlock" << endl;
  }
};

class CMutex
{
public:
  CMutex()
  {
    pthread_mutex_init(&m_mutex, NULL);
  }

  ~CMutex()
  {
    pthread_mutex_destroy(&m_mutex);
  }

  void Lock() const
  {
    pthread_mutex_lock(&m_mutex);
  }

  void Unlock() const
  {
    pthread_mutex_unlock(&m_mutex);
  }

private:
  // mute
  mutable pthread_mutex_t m_mutex;
};
}

#endif  // SPARK_BASE_MYLOCK_H_
