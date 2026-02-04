/*
 * mode_autonav.cpp
 *
 *  Created on: 2021年8月26日
 *      Author: 25053
 */
#include "maincontroller.h"

static float target_yaw=0.0f;
static bool jump=true;
static bool use_gcs=false, use_rc=false;
static Vector3f ned_target_pos;
static Vector2f ned_target_dis_2d, ned_dis_2d, vel_desired;
static uint32_t takeoff_time=0, lock_time=0, safe_time=0;
static float yaw_delta=0.0f;
static float jump_alt=0.0f;
static float takeoff_alt=0.0f;
static bool hit_target_takeoff_alt=true;
static float landing_alt=0.0f;
static float relative_alt=0.0f;
static Vector2f goal_2d;
static bool goal_reset=false, goal_set=false;
static bool use_surface_track=true;
static int16_t esc_counter=0, esc_delay=0;
bool mode_autonav_init(void){
	if(motors->get_armed()){//电机未锁定,禁止切换至该模式
		Buzzer_set_ring_type(BUZZER_ERROR);
		return false;
	}
	if(!pos_control->is_active_z()){
		// initialize position and desired velocity
		pos_control->set_alt_target_to_current_alt();
		pos_control->set_desired_velocity_z(get_vel_z());
	}
	set_manual_throttle(false);//设置为自动油门
	attitude->bf_feedforward(true);//是否启用姿态的前馈
	Buzzer_set_ring_type(BUZZER_MODE_SWITCH);
	usb_printf("switch mode autonav!\n");
	float ch7=get_channel_7();
	if(ch7>=0.7&&ch7<=1.0){//姿态模式
		usb_printf("enter submode attitude!\n");
	}else if(ch7>0.3&&ch7<0.7){//位置模式
		usb_printf("enter submode position!\n");
	}else{
		usb_printf("enter submode auto!\n");
	}
	return true;
}

void mode_autonav(void){
	AltHoldModeState althold_state;
	float takeoff_climb_rate = 0.0f;
	float ch7=get_channel_7();

	// initialize vertical speeds and acceleration
	pos_control->set_speed_z(-param->pilot_speed_dn.value, param->pilot_speed_up.value);
	pos_control->set_accel_z(param->pilot_accel_z.value);
	pos_control->set_speed_xy(param->poshold_vel_max.value);
	pos_control->set_accel_xy(param->poshold_accel_max.value);

	// get pilot desired lean angles
	float target_roll, target_pitch;
	get_pilot_desired_lean_angles(target_roll, target_pitch, param->angle_max.value, attitude->get_althold_lean_angle_max());

	// get pilot's desired yaw rate
	float target_yaw_rate = get_pilot_desired_yaw_rate(get_channel_yaw_angle());
	yaw_delta=-param->uwb_yaw_delta_deg.value*DEG_TO_RAD;
	// get pilot desired climb rate
	float target_climb_rate = get_pilot_desired_climb_rate(get_channel_throttle());
	target_climb_rate = constrain_float(target_climb_rate, -param->pilot_speed_dn.value, param->pilot_speed_up.value);

	if(get_force_autonav()&&!rc_channels_healthy()){//强制飞控进入自主模式
		ch7=-1.0f;
	}

	if(ch7<0.3f){//遥控器未连接或ROS控制
		target_roll=0.0f;
		target_pitch=0.0f;
		target_yaw_rate=0.0f;
		target_climb_rate=0.0f;
	}

	if((HAL_GetTick()-takeoff_time)>200&&takeoff_time>0){
		if(get_vib_value()>10.0){//猛烈撞击
			disarm_motors();
		}

		if(get_dcm_matrix().c.z>0.5f&&get_accel_filt().z<0){//倾角小于60度
			safe_time=HAL_GetTick();
		}else{
			if((HAL_GetTick()-safe_time)>3000){//大倾角
				disarm_motors();
			}
			if(get_dcm_matrix().c.z<0.0f){//倾倒
				disarm_motors();
			}
		}
	}

	// Alt Hold State Machine Determination
	if (!motors->get_armed()) {
		althold_state = AltHold_MotorStopped;
	} else if (takeoff_running() || takeoff_triggered(target_climb_rate) || get_takeoff()) {
		althold_state = AltHold_Takeoff;
	} else if (ap->land_complete) {
		althold_state = AltHold_Landed;
	} else {
		althold_state = AltHold_Flying;
	}

	// Alt Hold State Machine
	switch (althold_state) {

	case AltHold_MotorStopped:
		robot_state=STATE_STOP;
		lock_time=HAL_GetTick();
		takeoff_time=0;
		if(robot_state_desired==STATE_FLYING||robot_state_desired==STATE_TAKEOFF){
			arm_motors();
		}else{
			robot_state_desired=STATE_NONE;
		}
		if(!motors->get_interlock()){
			ekf_z_reset();
		}
		motors->set_desired_spool_state(Motors::DESIRED_SHUT_DOWN);
		attitude->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
		attitude->reset_rate_controller_I_terms();
		attitude->set_yaw_target_to_current_heading();
		target_yaw=log_yaw_deg();
		pos_control->set_xy_target(get_pos_x(), get_pos_y());
		pos_control->reset_predicted_accel(get_vel_x(), get_vel_y());
		pos_control->relax_alt_hold_controllers(0.0f);   // forces throttle output to go to zero
		pos_control->update_z_controller(get_pos_z(), get_vel_z());
		if(get_channel_5()<0.3){
			if(motors_test_update()){
				return;
			}
			/*
			 * 以下为电调校准模式说明:
			 * (1) 在电机锁定状态下,首先进行硬件解锁(注意不要进行手势解锁), 硬件解锁后Mcontroller右侧绿色指示灯长亮;
			 * (2) 将油门推到最高, 偏航推到最左, 持续5s进入电调校准模式;
			 * (3) 进入电调校准模式后提示音“嘟嘟嘟...”响起。此时将偏航回中, Mcontroller M1~M8插口会产生PWM输出;
			 * (4) 在电调校准模式中, Mcontroller M1~M8口输出的PWM波脉宽直接由油门推杆控制, 即最大油门对应最大脉宽, 最小油门对应最小脉宽。
			 * (5) 电调校准模式默认持续时间为50s, 即进入电调校准模式50s后自动退出电调校准模式;
			 * (6) 在电调校准模式中, 将偏航推到最右可以立即退出电调校准模式;
			 * */
			float throttle=get_channel_throttle();
			float tmp = get_channel_yaw();
			if (tmp < -0.9) { //full left
				// wait for 5s to enter esc correct mode
				if( throttle > 0.9 && esc_counter < 2000) {
					esc_counter++;
				}
				if (esc_counter == 2000) {
					esc_delay=20000; //设置电调校准持续时间为50s, 50s后自动退出电调校准模式
					Buzzer_set_ring_type(BUZZER_ESC);
				}
			}else if(tmp>0.5){	// Yaw is right to reset esc counter
				esc_counter = 0;
			}else{				// Yaw is centered to delay 20s to reset esc counter
				// correct the esc
				if(esc_delay>0){
					if (esc_counter == 2000) {
						Buzzer_set_ring_type(BUZZER_ESC);
						motors->set_throttle_passthrough_for_motors(throttle);//只校准当前机型使能的电机
					}
					esc_delay--;
					return;
				}else{
					esc_counter = 0;
					esc_delay = 0;
				}
			}
		}
		break;

	case AltHold_Takeoff:
		robot_state=STATE_TAKEOFF;
		// set motors to full range
		motors->set_desired_spool_state(Motors::DESIRED_THROTTLE_UNLIMITED);
		if(HAL_GetTick()-lock_time<2000){
			break;
		}
		motors->set_throttle_hover(0.35);//设置悬停油门, 需要根据不同的机型自行调整
		takeoff_alt=constrain_float(param->pilot_takeoff_alt.value,30.0f,200.0f);//最小30cm,最大200cm
		// initiate take-off
		if (!takeoff_running()) {
			if(get_batt_volt()<param->lowbatt_land_volt.value){
				disarm_motors();
				break;
			}
			takeoff_start(takeoff_alt);
			hit_target_takeoff_alt=false;
			// indicate we are taking off
			set_land_complete(false);
			// clear i terms
			set_throttle_takeoff();
			landing_alt=get_pos_z();
			set_target_rangefinder_alt(landing_alt+takeoff_alt);
			reset_mav_target_state();
			if(jump){//起飞时直接跳起
				pos_control->get_accel_z_pid().set_integrator(0.0f);
				pos_control->set_alt_target(get_pos_z()+jump_alt);//设置目标高度比当前高度高jump_alt
			}
			takeoff_time=HAL_GetTick();
			if(get_gcs_connected()){
				use_gcs=true;
			}else{
				use_gcs=false;
			}
			if(rc_channels_healthy()){
				use_rc=true;
			}else{
				use_rc=false;
			}
		}

		// get take-off adjusted pilot and takeoff climb rates
		if(ch7<0.3f||target_climb_rate>-20.0f){
			target_climb_rate=MAX(target_climb_rate, param->auto_takeoff_speed.value);//给一个初速度,大飞机30,小飞机50
		}
		get_takeoff_climb_rates(target_climb_rate, takeoff_climb_rate);

		if(!hit_target_takeoff_alt){
			if(rangefinder_state.alt_healthy&&abs(rangefinder_state.alt_cm + get_vel_z()*0.2 - takeoff_alt)<10.0f){
				hit_target_takeoff_alt=true;
				takeoff_stop();
			}
		}

		// call attitude controller
		if(ch7>=0.7&&ch7<=1.0){//姿态模式
			robot_spec_mode=MODE_ATTITUDE;
			target_yaw+=target_yaw_rate*_dt;
			attitude->input_euler_angle_roll_pitch_yaw(target_roll, target_pitch, target_yaw, true);
			pos_control->set_xy_target(get_pos_x(), get_pos_y());
			pos_control->reset_predicted_accel(get_vel_x(), get_vel_y());
		}else{//位置模式
			robot_spec_mode=MODE_POSITION;
			target_yaw+=target_yaw_rate*_dt;
			get_accel_vel_limit();
			pos_control->set_pilot_desired_acceleration(target_roll, target_pitch, target_yaw, _dt);
			pos_control->calc_desired_velocity(_dt);
			pos_control->update_xy_controller(_dt, get_pos_x(), get_pos_y(), get_vel_x(), get_vel_y());
			target_roll=pos_control->get_roll();
			target_pitch=pos_control->get_pitch();
			attitude->input_euler_angle_roll_pitch_yaw(target_roll, target_pitch, target_yaw, true);
		}
		// call position controller
		pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, _dt, false);
		pos_control->add_takeoff_climb_rate(takeoff_climb_rate, _dt);
		pos_control->update_z_controller(get_pos_z(), get_vel_z());
		break;

	case AltHold_Landed:
		robot_state=STATE_LANDED;
		takeoff_time=0;
		// set motors to spin-when-armed if throttle below deadzone, otherwise full range (but motors will only spin at min throttle)
		if (target_climb_rate < 0.0f) {
			motors->set_desired_spool_state(Motors::DESIRED_SPIN_WHEN_ARMED);
		} else {
			motors->set_desired_spool_state(Motors::DESIRED_THROTTLE_UNLIMITED);
		}
		attitude->set_yaw_target_to_current_heading();
		target_yaw=log_yaw_deg();
		attitude->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
		attitude->reset_rate_controller_I_terms();
		pos_control->set_xy_target(get_pos_x(), get_pos_y());
		pos_control->reset_predicted_accel(get_vel_x(), get_vel_y());
		pos_control->relax_alt_hold_controllers(0.0f);   // forces throttle output to go to zero
		pos_control->update_z_controller(get_pos_z(), get_vel_z());
		break;

	case AltHold_Flying:
		robot_state=STATE_FLYING;
		motors->set_desired_spool_state(Motors::DESIRED_THROTTLE_UNLIMITED);

		// call attitude controller
		if((use_gcs||use_rc)&&!rc_channels_healthy()&&!get_gcs_connected()){
			robot_state_desired=STATE_LANDED;
			target_roll=0.0f;
			target_pitch=0.0f;
			target_yaw_rate=0.0f;
			ch7=constrain_float(ch7, 0.5, 0.9);//强制在位置模式下降落
		}

		if((HAL_GetTick()-takeoff_time)<10000){//起飞10s内禁止glitch
			rangefinder_state.glitch_count=0;
		}

		if(ch7>=0.7&&ch7<=1.0){//姿态模式
			robot_spec_mode=MODE_ATTITUDE;
			target_yaw+=target_yaw_rate*_dt;
			attitude->input_euler_angle_roll_pitch_yaw(target_roll, target_pitch, target_yaw, true);
			pos_control->set_xy_target(get_pos_x(), get_pos_y());
			pos_control->reset_predicted_accel(get_vel_x(), get_vel_y());
		}else if(ch7>0.3&&ch7<0.7){//位置模式
			robot_spec_mode=MODE_POSITION;
			target_yaw+=target_yaw_rate*_dt;
			get_accel_vel_limit();
			pos_control->set_pilot_desired_acceleration(target_roll, target_pitch, target_yaw, _dt);
			pos_control->calc_desired_velocity(_dt);
			pos_control->update_xy_controller(_dt, get_pos_x(), get_pos_y(), get_vel_x(), get_vel_y());
			target_roll=pos_control->get_roll();
			target_pitch=pos_control->get_pitch();
			attitude->input_euler_angle_roll_pitch_yaw(target_roll, target_pitch, target_yaw, true);
		}else{//自主模式
			robot_spec_mode=MODE_AUTO;
			if((HAL_GetTick()-takeoff_time)<2000){
				get_accel_vel_limit();
				pos_control->update_xy_controller(_dt, get_pos_x(), get_pos_y(), get_vel_x(), get_vel_y());
				target_roll=pos_control->get_roll();
				target_pitch=pos_control->get_pitch();
				attitude->input_euler_angle_roll_pitch_yaw(target_roll, target_pitch, target_yaw, true);
			}else{
				get_accel_vel_limit();
				if(get_mav_target_state()){
					target_yaw_rate=get_mav_yaw_rate_target();
					if(target_yaw_rate==0.0f){
						target_yaw=(get_mav_yaw_target()*DEG_TO_RAD-yaw_delta)*RAD_TO_DEG;
					}else{
						target_yaw+=target_yaw_rate*_dt;
					}
					switch(get_coordinate_mode()){
						case MAV_FRAME_BODY_NED:
							pos_control->set_pilot_desired_acceleration(get_mav_ax_roll_target(), get_mav_ay_pitch_target(), target_yaw, _dt);
							pos_control->calc_desired_velocity(_dt);
							relative_alt=get_mav_z_target();
							break;
						case MAV_FRAME_VISION_NED:
							if(get_mav_x_target()!=0.0f||get_mav_y_target()!=0.0f){
								pos_control->set_xy_target(get_mav_x_target(),get_mav_y_target());
							}
							pos_control->set_desired_velocity_xy(get_mav_vx_target(), get_mav_vy_target());
							pos_control->set_desired_accel_xy(get_mav_ax_target(), get_mav_ay_target());
							if(get_mav_z_target()!=0.0f){
								relative_alt=get_mav_z_target();
							}
							target_climb_rate=get_mav_vz_target();
							break;
						case MAV_FRAME_LOCAL_NED:
							ned_target_pos.x=get_mav_x_target()*cosf(yaw_delta)+get_mav_y_target()*sinf(yaw_delta);
							ned_target_pos.y=-get_mav_x_target()*sinf(yaw_delta)+get_mav_y_target()*cosf(yaw_delta);
							pos_control->set_xy_target(ned_target_pos.x,ned_target_pos.y);
							pos_control->set_desired_velocity_xy(0.0f, 0.0f);
							pos_control->set_desired_accel_xy(0.0f, 0.0f);
							relative_alt=get_mav_z_target();
							break;
						case MAV_FRAME_GLOBAL:
							ned_target_pos.x=get_mav_x_target()*cosf(yaw_delta)+get_mav_y_target()*sinf(yaw_delta);
							ned_target_pos.y=-get_mav_x_target()*sinf(yaw_delta)+get_mav_y_target()*cosf(yaw_delta);
							ned_dis_2d.x=ned_target_pos.x-pos_control->get_pos_target().x;//重新计算当前目标与上一个目标点的距离
							ned_dis_2d.y=ned_target_pos.y-pos_control->get_pos_target().y;
							if(ned_dis_2d.length()>1.0f){
								vel_desired=ned_dis_2d.normalized()*param->mission_vel_max.value;//设置跟踪速度
								pos_control->shift_pos_xy_target(vel_desired.x*_dt, vel_desired.y*_dt);
							}else{
								pos_control->set_xy_target(ned_target_pos.x,ned_target_pos.y);
							}
							pos_control->set_desired_velocity_xy(0.0f, 0.0f);
							pos_control->set_desired_accel_xy(0.0f, 0.0f);
							relative_alt=get_mav_z_target();
							break;
						default:
							pos_control->set_desired_velocity_xy(0.0f, 0.0f);
							pos_control->set_desired_accel_xy(0.0f, 0.0f);
							break;
					}

					if(robot_state_desired!=STATE_LANDED&&relative_alt!=0.0f){
						if(use_surface_track){
							if(!USE_ODOM_Z){
								relative_alt=MAX(relative_alt,30.0f);
							}
							set_target_rangefinder_alt(relative_alt+landing_alt);
						}
						if(!rangefinder_state.alt_healthy||!use_surface_track){
							pos_control->set_alt_target(relative_alt+landing_alt);
						}
					}
				}else{
					pos_control->set_desired_velocity_xy(0.0f, 0.0f);
					pos_control->set_desired_accel_xy(0.0f, 0.0f);
				}
				float stick=safe_sqrt(get_channel_pitch()*get_channel_pitch() + get_channel_roll()*get_channel_roll());
				if(stick>0.9f){
					if(goal_reset){
						goal_set=true;
						float goal_fwd=-get_channel_pitch();
						float goal_rgt=get_channel_roll();
						goal_2d.x = (goal_fwd*ahrs_cos_yaw() - goal_rgt*ahrs_sin_yaw());
						goal_2d.y = (goal_fwd*ahrs_sin_yaw() + goal_rgt*ahrs_cos_yaw());
						goal_2d=goal_2d.normalized()*10.0f*constrain_float(get_channel_8(), 0.1f, 1.0f);//默认单次探索范围1~10m
						goal_2d.x+=get_pos_x()*0.01;
						goal_2d.y+=get_pos_y()*0.01;
						goal_reset=false;
					}
				}else if(stick<0.1f){
					if(goal_set){
						send_mavlink_goal_point(goal_2d.x, goal_2d.y, get_rangefinder_alt_target()*0.01f);
					}
					goal_reset=true;
					goal_set=false;
				}
				pos_control->update_xy_controller(_dt, get_pos_x(), get_pos_y(), get_vel_x(), get_vel_y());
				target_roll=pos_control->get_roll();
				target_pitch=pos_control->get_pitch();
				attitude->input_euler_angle_roll_pitch_yaw(target_roll, target_pitch, target_yaw, true);
			}
		}

		if(get_batt_volt()<param->lowbatt_land_volt.value&&(HAL_GetTick()-takeoff_time)>2000){//电量过低，强制降落
			robot_state_desired=STATE_LANDED;
		}

		if(robot_state_desired==STATE_LANDED){//自动降落
			target_climb_rate=-constrain_float(param->auto_land_speed.value, 0.0f, param->pilot_speed_dn.value);//设置降落速度cm/s
		}

		// adjust climb rate using rangefinder
		if(!use_ego_mission()||USE_ODOM_Z){
			target_climb_rate = get_surface_tracking_climb_rate(target_climb_rate, pos_control->get_alt_target(), _dt);
			use_surface_track=true;
		}else{
			use_surface_track=false;
		}

		// call position controller
		pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, _dt, false);

		pos_control->update_z_controller(get_pos_z(), get_vel_z());
		break;
	default:
		break;
	}
	attitude->rate_controller_run();
	motors->output();
}
