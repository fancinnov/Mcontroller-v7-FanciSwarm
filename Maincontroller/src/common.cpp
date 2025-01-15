/*
 * common.cpp
 *
 *  Created on: 2020.07.16
 *      Author: JackyPan
 */
#include "maincontroller.h"

static bool _soft_armed=false;//心跳包中表示是否解锁的标志位
static bool _thr_force_decrease=false;//强制油门下降
static bool _vel_d_constrain=false;
ROBOT_STATE robot_state=STATE_NONE;
ROBOT_STATE robot_state_desired=STATE_NONE;
ROBOT_MAIN_MODE robot_main_mode=MODE_AIR;
ROBOT_SUB_MODE robot_sub_mode=MODE_STABILIZE;

bool get_soft_armed(void){
	return _soft_armed;
}

void set_soft_armed(bool soft_armed){
	_soft_armed=soft_armed;
}

bool get_thr_force_decrease(void){
	return _thr_force_decrease;
}

void set_thr_force_decrease(bool force_decrease){
	_thr_force_decrease=force_decrease;
}

bool get_constrain_vel_d(void){
	return _vel_d_constrain;
}

void set_constrain_vel_d(bool constrain){
	_vel_d_constrain=constrain;
}

bool mode_init(void){
	if(robot_sub_mode!=MODE_AUTONAV){
		if(mode_autonav_init()){
			robot_sub_mode=MODE_AUTONAV;
			return true;
		}else{
			return false;
		}
	}else{
		return true;
	}
}

//should be run at 400hz
void mode_update(void){
	if(mode_init()){
		mode_autonav();
	}
}
