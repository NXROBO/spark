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

#ifndef SPARK_BASE_KFILTER_H_
#define SPARK_BASE_KFILTER_H_

#include <math.h>
#include <iostream>
using namespace std;

namespace nxsparkbase
{
class KFilter
{
private:
  // initial values for the kalman filter
  double x_est_last;
  double P_last;

  // the noise in the system
  double Q;
  double R;

  double K;
  double P;
  double P_temp;
  double x_temp_est;
  double x_est;
  double z_measured;  // the 'noisy' value we measured
public:
  KFilter()
  {
    // initial values for the kalman filter
    x_est_last = 0;
    P_last = 0;

    Q = 0.001;
    R = 0.1;
    x_temp_est = 0;
    x_est_last = 0;
  }

  /**
   *
   */
  float predict(double x)
  {
    // do a prediction
    x_temp_est = x_est_last;
    P_temp = P_last + Q;
    // calculate the Kalman gain
    K = P_temp * (1.0 / (P_temp + R));
    // measure
    z_measured = x;  // the real measurement plus noise
    // correct
    x_est = x_temp_est + K * (z_measured - x_temp_est);
    P = (1 - K) * P_temp;
    // we have our new system

    // update our last's
    P_last = P;
    x_est_last = x_est;

    return x_est;
  }
};
}

#endif  // SPARK_BASE_KFILTER_H_

/*
void main()
{
        float y_real;
        float y_measure;
        float y_est;
        kfilter kf;
        for(int i = 0; i < 10000;i++)
        {
                y_real = std::sin(i/20.0);
                //y_real = 2;
                y_measure = y_real + (std::rand()%100-50)*0.001;
                y_est = kf.predict(y_measure);
                cout<<y_measure<<","<<y_real<<","<<y_est<<endl;
                Sleep(10);
        }
        return;
}*/
