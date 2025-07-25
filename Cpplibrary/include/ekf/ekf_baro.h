/*
 * ekf_baro.h
 *
 *  Created on: 2021年7月9日
 *      Author: JackyPan
 */
#pragma once

#ifndef INCLUDE_EKF_EKF_BARO_H_
#define INCLUDE_EKF_EKF_BARO_H_

#include "common.h"

class EKF_Baro{

public:
	EKF_Baro(float dt, float Q1, float Q2, float R1, float R2);
	void update(bool &get_baro_alt_filt, float baro_alt);
	void fusion(float baro_alt, float &baro_alt_correct);
	float pos_z=0, vel_z=0, vel_2d=0, accel_2d=0, accelz_ef=0;
	float get_vt(void){return vt_last;}
	bool is_initialed(void){return initialed;}
	bool use_odom_z=false;
	float gravity=GRAVITY_MSS;
	void reset(void){
		initialed_fusion=false;
		initialed=false;
	}

private:
	float Qt[2*2]={0.0016, 		0,
					0,     0.0016};//观测数据的方差
	float _filt_alpha(float dt, float filt_hz);
	float _alpha=0;
	bool initialed=false, initialed_fusion=false;
	float delta_x=0, delta_v=0;
	float T_baro=0.0025; //2.5ms
	float G[2*2]={ 1, T_baro,
				   0, 	1};
	float GT[2*2]={1,	   0,
				   T_baro, 1};
	float h_x=0, h_v=0, zt=0, zt_last=0, vt=0, vt_last=0, t_now=0, t_last=0;
	float error_x, error_v;
	float error_xv[2*2]={1,0,
				 	 	 0,1};
	float inv[4];
	float H[2*2]={1,0,
				  0,1};
	float HT[2*2]={1,0,
					0,1};
	float Rt[2*2]={ 0.000016,       0,	//预测数据x方差
					0,          0.000016};	//预测数据v方差
	float error[2*2]={  1.0,        0,
					    0,          1.0};
	float error_p[2*2];
	float* error1;
	float* error2;
	float* Kal;
	DerivativeFilterFloat_Size7 _climb_rate_filter;
	float alt_last=0;
	float baro_alt_last=0,gnss_alt_last=0;
	float rf_alt_delta=0, rf_alt_last=0, gnss_alt_delta=0, baro_alt_delta=0, baro_alt_offset=0;
	float gnss_vel_z=0.0f;
	float K_gain=0.0f;
	bool rf_correct=false;
	float baro_alt_init=0.0f;
	float baro_alt_real=0.0f;
	float pos_z_last=0.0f;
};
#endif /* INCLUDE_EKF_EKF_BARO_H_ */
