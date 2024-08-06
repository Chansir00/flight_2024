/* Copyright (c)  2018-2025 Wuhan Nameless Innovation Technology Co.,Ltd. All rights reserved.*/
/*----------------------------------------------------------------------------------------------------------------------/
                开源并不等于免费，先驱者的历史已经证明，在国内这个环境下，毫无收益的开源，单靠坊间个人爱好者，自发地参与项
								目完善的方式行不通，好的开源项目需要请专职人员做好售后服务、手把手教学、统计用户反馈需求、在实践中完成对产
								品的一次次完善与迭代升级。经过综合考虑，无名飞控开源代码中，程序仅保留公司正版激活功能代码，版本激活无实际
								功能，属于公司产品常规出厂操作，不影响客户学习，其余代码全部开放给客户学习，客户移植和二次开发代码请保留代
								码版权。
-----------------------------------------------------------------------------------------------------------------------/
*               本程序只供购买者学习使用，版权著作权属于无名科创团队，无名科创团队将飞控程序源码提供给购买者，
*               购买者要为无名科创团队提供保护，未经作者许可，不得将源代码提供给他人，不得将源代码放到网上供他人免费下载， 
*               更不能以此销售牟利，如发现上述行为，无名科创团队将诉之以法律解决！！！
-----------------------------------------------------------------------------------------------------------------------
*                                                 为什么选择无名创新？
*                                         感动人心价格厚道，最靠谱的开源飞控；
*                                         国内业界良心之作，最精致的售后服务；
*                                         追求极致用户体验，高效进阶学习之路；
*                                         萌新不再孤单求索，合理把握开源尺度；
*                                         响应国家扶贫号召，促进教育体制公平；
*                                         新时代奋斗最出彩，建人类命运共同体。 
-----------------------------------------------------------------------------------------------------------------------
*               生命不息、奋斗不止；前人栽树，后人乘凉！！！
*               开源不易，且学且珍惜，祝早日逆袭、进阶成功！！！
*               学习优秀者，简历可推荐到DJI、ZEROTECH、XAG、AEE、GDU、AUTEL、EWATT、HIGH GREAT等公司就业
*               求职简历请发送：15671678205@163.com，需备注求职意向单位、岗位、待遇等
*               无名科创开源飞控QQ群：2号群465082224、1号群540707961（人员已满）
*               CSDN博客：http://blog.csdn.net/u011992534
*               优酷ID：NamelessCotrun无名小哥
*               B站教学视频：https://space.bilibili.com/67803559/#/video
*               客户使用心得、改进意见征集贴：http://www.openedv.com/forum.php?mod=viewthread&tid=234214&extra=page=1
*               淘宝店铺：https://shop348646912.taobao.com/?spm=2013.1.1000126.2.5ce78a88ht1sO2
*               百度贴吧:无名科创开源飞控
*               公司官网:www.nameless.tech
*               无名创新国内首款TI开源飞控设计初衷、知乎专栏:https://zhuanlan.zhihu.com/p/54471146
*               修改日期:2021/04/30                    
*               版本：筑梦者PLUS——GankerPilot_V3.0
*               版权所有，盗版必究。
*               Copyright(C) 2019-2025 武汉无名创新科技有限公司 
*               All rights reserved
-----------------------------------------------------------------------------------------------------------------------
*               重要提示：
*               正常淘宝咸鱼转手的飞控、赠送朋友、传给学弟的都可以进售后群学习交流，
*               不得直接在网上销售无名创新资料，无名创新代码有声明版权，他人不得将
*               资料代码传网上供他人下载，不得以谋利为目的销售资料代码，发现有此操
*               作者，公司会提前告知，请1天内及时处理，否则你的学校、单位、姓名、电
*               话、地址信息会被贴出在公司官网、官方微信公众平台、官方技术博客、知乎
*               专栏以及淘宝店铺首页予以公示公告，此种所作所为，会成为个人污点，影响
*               升学、找工作、社会声誉、很快就很在无人机界出名，后果很严重。
*               因此行为给公司造成重大损失者，会以法律途径解决，感谢您的合作，谢谢！！！
----------------------------------------------------------------------------------------------------------------------*/
#include "madgwick.h"
#include "wp_math.h"
#include <math.h>


float _beta;				  // algorithm gain
float _q0,_q1,_q2,_q3;	// quaternion of sensor frame relative to auxiliary frame
float invSampleFreq;
float roll,pitch,yaw;
#define betaDef         0.1f            // 2 * proportional gain

void madgwick_init() 
{
	_beta = betaDef;
	_q0 = 1.0f;
	_q1 = 0.0f;
	_q2 = 0.0f;
	_q3 = 0.0f;
}

void madgwick_update(float *gyro_r,float *accel_g,float *mag_t,float dt) 
{
	float gx=gyro_r[0];
	float gy=gyro_r[1];
  float gz=gyro_r[2];
	float ax=accel_g[0];
	float ay=accel_g[1];
  float az=accel_g[2];
	float mx=mag_t[0];
	float my=mag_t[1];
  float mz=mag_t[2];
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float hx, hy;
	float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
		madgwick_updateimu(gyro_r,accel_g,dt);
		return;
	}

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-_q1 * gx - _q2 * gy - _q3 * gz);
	qDot2 = 0.5f * (_q0 * gx + _q2 * gz - _q3 * gy);
	qDot3 = 0.5f * (_q0 * gy - _q1 * gz + _q3 * gx);
	qDot4 = 0.5f * (_q0 * gz + _q1 * gy - _q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = madgwick_invsqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Normalise magnetometer measurement
		recipNorm = madgwick_invsqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		_2q0mx = 2.0f * _q0 * mx;
		_2q0my = 2.0f * _q0 * my;
		_2q0mz = 2.0f * _q0 * mz;
		_2q1mx = 2.0f * _q1 * mx;
		_2q0 = 2.0f * _q0;
		_2q1 = 2.0f * _q1;
		_2q2 = 2.0f * _q2;
		_2q3 = 2.0f * _q3;
		_2q0q2 = 2.0f * _q0 * _q2;
		_2q2q3 = 2.0f * _q2 * _q3;
		q0q0 = _q0 * _q0;
		q0q1 = _q0 * _q1;
		q0q2 = _q0 * _q2;
		q0q3 = _q0 * _q3;
		q1q1 = _q1 * _q1;
		q1q2 = _q1 * _q2;
		q1q3 = _q1 * _q3;
		q2q2 = _q2 * _q2;
		q2q3 = _q2 * _q3;
		q3q3 = _q3 * _q3;

		// Reference direction of Earth's magnetic field
		hx = mx * q0q0 - _2q0my * _q3 + _2q0mz * _q2 + mx * q1q1 + _2q1 * my * _q2 + _2q1 * mz * _q3 - mx * q2q2 - mx * q3q3;
		hy = _2q0mx * _q3 + my * q0q0 - _2q0mz * _q1 + _2q1mx * _q2 - my * q1q1 + my * q2q2 + _2q2 * mz * _q3 - my * q3q3;
		_2bx = sqrtf(hx * hx + hy * hy);
		_2bz = -_2q0mx * _q2 + _2q0my * _q1 + mz * q0q0 + _2q1mx * _q3 - mz * q1q1 + _2q2 * my * _q3 - mz * q2q2 + mz * q3q3;
		_4bx = 2.0f * _2bx;
		_4bz = 2.0f * _2bz;

		// Gradient decent algorithm corrective step
		s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * _q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * _q3 + _2bz * _q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * _q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * _q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * _q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * _q2 + _2bz * _q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * _q3 - _4bz * _q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * _q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * _q2 - _2bz * _q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * _q1 + _2bz * _q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * _q0 - _4bz * _q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * _q3 + _2bz * _q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * _q0 + _2bz * _q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * _q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		recipNorm = madgwick_invsqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= _beta * s0;
		qDot2 -= _beta * s1;
		qDot3 -= _beta * s2;
		qDot4 -= _beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	_q0 += qDot1 * dt;
	_q1 += qDot2 * dt;
	_q2 += qDot3 * dt;
	_q3 += qDot4 * dt;

	// Normalise quaternion
	recipNorm = madgwick_invsqrt(_q0 * _q0 + _q1 * _q1 + _q2 * _q2 + _q3 * _q3);
	_q0 *= recipNorm;
	_q1 *= recipNorm;
	_q2 *= recipNorm;
	_q3 *= recipNorm;
}

//-------------------------------------------------------------------------------------------
// IMU algorithm update

void madgwick_updateimu(float *gyro_r,float *accel_g,float dt)
{
	static uint8_t init_flag=0;
	if(init_flag==0)
	{
		init_flag=1;
	  madgwick_init();
	  return ;
	}
	float gx=gyro_r[0];
	float gy=gyro_r[1];
  float gz=gyro_r[2];
	float ax=accel_g[0];
	float ay=accel_g[1];
  float az=accel_g[2];
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-_q1 * gx - _q2 * gy - _q3 * gz);
	qDot2 = 0.5f * (_q0 * gx + _q2 * gz - _q3 * gy);
	qDot3 = 0.5f * (_q0 * gy - _q1 * gz + _q3 * gx);
	qDot4 = 0.5f * (_q0 * gz + _q1 * gy - _q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = madgwick_invsqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0f * _q0;
		_2q1 = 2.0f * _q1;
		_2q2 = 2.0f * _q2;
		_2q3 = 2.0f * _q3;
		_4q0 = 4.0f * _q0;
		_4q1 = 4.0f * _q1;
		_4q2 = 4.0f * _q2;
		_8q1 = 8.0f * _q1;
		_8q2 = 8.0f * _q2;
		q0q0 = _q0 * _q0;
		q1q1 = _q1 * _q1;
		q2q2 = _q2 * _q2;
		q3q3 = _q3 * _q3;

		// Gradient decent algorithm corrective step
		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
		s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * _q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		s2 = 4.0f * q0q0 * _q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
		s3 = 4.0f * q1q1 * _q3 - _2q1 * ax + 4.0f * q2q2 * _q3 - _2q2 * ay;
		recipNorm = madgwick_invsqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= _beta * s0;
		qDot2 -= _beta * s1;
		qDot3 -= _beta * s2;
		qDot4 -= _beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	_q0 += qDot1 * dt;
	_q1 += qDot2 * dt;
	_q2 += qDot3 * dt;
	_q3 += qDot4 * dt;

	// Normalise quaternion
	recipNorm = madgwick_invsqrt(_q0 * _q0 + _q1 * _q1 + _q2 * _q2 + _q3 * _q3);
	_q0 *= recipNorm;
	_q1 *= recipNorm;
	_q2 *= recipNorm;
	_q3 *= recipNorm;
}

//-------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

float madgwick_invsqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	y = y * (1.5f - (halfx * y * y));
	return y;
}

//-------------------------------------------------------------------------------------------
void madgwick_getangle(float* rpy)
{
	pitch= RADTODEG(atan2f(_q0*_q1 + _q2*_q3, 0.5f - _q1*_q1 - _q2*_q2));
	roll = RADTODEG(asinf(-2.0f * (_q1*_q3 - _q0*_q2)));
	yaw  = RADTODEG(atan2f(_q1*_q2 + _q0*_q3, 0.5f - _q2*_q2 - _q3*_q3));
	
	rpy[0] = roll;
	rpy[1] = pitch;
	rpy[2] = yaw;	
}

