//=============================================================================================
// MadgwickAHRS.h
//=============================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
//
// From the x-io website "Open-source resources available on this website are
// provided under the GNU General Public Licence unless an alternative licence
// is provided in source."
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=============================================================================================
#ifndef _MADGWICK_H_
#define _MADGWICK_H_
#include <math.h>

void madgwick_init(void);
void madgwick_updateimu(float *gyro_r,float *accel_g,float dt);
void madgwick_update(float *gyro_r,float *accel_g,float *mag_t,float dt);
float madgwick_invsqrt(float x);
void madgwick_getangle(float* rpy);



extern float roll,pitch,yaw;
extern float _q0,_q1,_q2,_q3;
#endif

