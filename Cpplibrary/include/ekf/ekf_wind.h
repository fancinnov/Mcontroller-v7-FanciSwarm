/*
 * ekf_wind.h
 *
 *  Created on: 2024年2月23日
 *      Author: JackyPan
 */
#pragma once

#ifndef INCLUDE_EKF_EKF_WIND_H_
#define INCLUDE_EKF_EKF_WIND_H_

#include "common.h"

class EKF_Wind{

public:
	EKF_Wind(float dt, float Q, float R1, float R2);
	void update(bool get_vel_data, float vel_x, float vel_y);
	bool is_initialed(void){return initialed;}
	void reset(void){
		initialed=false;
	}
	float wind_x=0.0f, wind_y=0.0f, vx_body=0.0f, vy_body=0.0f;
	float wind_x_filt=0.0f, wind_y_filt=0.0f;
private:
	float accel_x_filt=0.0f, accel_y_filt=0.0f;
	float euler_roll_angle=0.0f,euler_pitch_angle=0.0f,pilot_cos_pitch_target=0.0f;
	float ax_body=0.0f,ay_body=0.0f,pilot_actual_accel_x=0.0f,pilot_actual_accel_y=0.0f;
	float Qt=1.0f; //观测数据的方差
	bool initialed=false;
	float _filt_alpha(float dt, float filt_hz);
	float _alpha=0, _alpha_accel=0;
	float delta_x_wind=0;
	float T_wind=0.0025; //2.5ms
	float G_wind[2*2]={ 1, T_wind,
				   0, 	1};
	float GT_wind[2*2]={1,	 0,
			T_wind, 1};
	float h_wind=0 ,error_x_wind=0, error_y_wind=0, zt_velx=0, zt_vely=0;
	float H_wind_x[1*2]={1,0};
	float HT_wind_x[2*1]={1,0};
	float Rt_wind_x[2*2]={ 0.000016,          0, 	//预测数据x方差
						0,              0.16}; 	//预测数据v方差
	float error_wind_x[2*2]={ 1.0,       0,
						  0,         1.0};
	float error_p_wind_x[2*2];
	float* error1_wind_x;
	float* error2_wind_x;
	float* Kal_wind_x;

	float delta_y_wind=0;
	float H_wind_y[1*2]={1,0};
	float HT_wind_y[2*1]={1,0};
	float Rt_wind_y[2*2]={ 0.000016,          0, 	//预测数据x方差
						0,              0.16}; 	//预测数据v方差
	float error_wind_y[2*2]={ 1.0,       0,
						  0,         1.0};
	float error_p_wind_y[2*2];
	float* error1_wind_y;
	float* error2_wind_y;
	float* Kal_wind_y;
	float filt_hz_wind=20.0f;
	float filt_hz_accel=5.0f;
};

#endif /* INCLUDE_EKF_EKF_WIND_H_ */
