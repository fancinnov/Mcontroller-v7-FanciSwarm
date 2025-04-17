/*
 * maincontroller.cpp
 *
 *  Created on: 2020.06.19
 *      Author: JackyPan
 */
#include "maincontroller.h"

/*************配置rangefinder ekf定高*************
 * 1.在freertos中启动ekf_rf_alt();
 * 2.更改get_pos_z()中的返回值为ekf_rangefinder->pos_z;
 * 3.更改get_vel_z()中的返回值为ekf_rangefinder->vel_z;
 * ****************************************/

/************Noted*************
 *
 * 控制不稳定,与传感器噪声、机架震动、机体质量分布有很大关系,
 * 需要调filter参数、ekf参数、pid参数、hover_throttle参数
 *
 * ****************************/

/* *************************************************
 * ****************Dev code begin*******************/
// Warning! Developer can add your new code here!

/* ****************Dev code end*********************
 * *************************************************/

static bool accel_cal_succeed=true;
static bool gyro_cal_succeed=false;
static bool compass_cal_succeed=false;
static bool ahrs_stage_compass=false;
static bool get_baro_alt_filt=false;
static bool initial_accel_gyro=false;
static bool initial_mag=false;
static bool initial_baro=false;
static bool calibrate_failure=false;
static bool initial_accel_cal=false;
static bool horizon_correct=false;
static bool reset_horizon_integrator=false;
static bool ahrs_healthy=false;
static bool initial_compass_cal=false;
static bool initial_gnss=false;
static bool get_gnss_location=false;
static bool get_opticalflow=false;
static bool get_rangefinder_data=false;
static bool get_mav_yaw=false, get_odom_xy=false, update_odom_xy=false;
static bool mag_corrected=false, mag_correcting=false;
static bool rc_channels_sendback=false;
static bool gcs_connected=false;
static bool offboard_connected=false;
static bool force_autonav=false;
static bool enable_surface_track=true;
static bool use_rangefinder=true;
static bool use_uwb=false;
static bool update_pos=false;
static bool enable_odom=true;
static bool odom_safe=true;
static bool get_mav_target=false;

static float accel_filt_hz=10;//HZ
static float gyro_filt_hz=20;//HZ
static float mag_filt_hz=5;//HZ
static float baro_filt_hz=5;//HZ
static float accel_ef_filt_hz=10;//HZ
static float uwb_pos_filt_hz=5;//HZ
static float rangefinder_filt_hz=20;//HZ
static float flow_gyro_filt_hz=5;//HZ
static float pitch_rad=0 , roll_rad=0 , yaw_rad=0;
static float pitch_deg=0 , roll_deg=0 , yaw_deg=0;
static float pitch_log=0 , roll_log=0 , yaw_log=0;
static float cos_roll=0, cos_pitch=0, cos_yaw=0, sin_roll=0, sin_pitch=0, sin_yaw=0;
static float yaw_map=0.0f;
static float mav_x_target=0.0f, mav_y_target=0.0f, mav_z_target=0.0f, mav_vx_target=0.0f, mav_vy_target=0.0f, mav_vz_target=0.0f, mav_yaw_target=0.0f,
		mav_ax_roll_target=0.0f, mav_ay_pitch_target=0.0f, mav_ax_target=0.0f, mav_ay_target=0.0f, mav_az_target=0.0f, mav_yaw_rate_target=0.0f;
static float completion_percent=0;
static float uwb_yaw_delta=0.0f;
static float takeoff_alt=0.0f;
static float odom_z_last=0.0f;
static float vins_tc=0.15f;//vins延迟0.15s
static float lidar_tc=0.05f;//lidar延迟0.05s
static float vins_dt=0.1f, lidar_dt=0.1f;//slam周期

static Vector3f accel, gyro, mag;								//原生加速度、角速度、磁罗盘测量值
static Vector3f accel_correct, gyro_correct, mag_correct;		//修正后的加速度、角速度、磁罗盘测量值
static Vector3f accel_filt, gyro_filt, mag_filt;				//滤波优化后的加速度、角速度、磁罗盘测量值
static Vector3f accel_ef, gyro_ef, accel_ef_filt;				//地球坐标系下的三轴加速度、角速度
static Vector3f gyro_offset;
static Vector3f odom_3d,odom_offset,uwb_pos;
static Location gnss_origin_pos, gnss_current_pos;
static Vector3f ned_current_pos, ned_current_vel;
static Matrix3f dcm_matrix, dcm_matrix_correct;										//旋转矩阵
static LowPassFilter2pVector3f	_accel_filter, _gyro_filter, _accel_ef_filter;
static LowPassFilterFloat _baro_alt_filter;
static LowPassFilterVector3f _mag_filter, _uwb_pos_filter, _flow_gyro_filter;
static Logger_Status m_Logger_Status= Logger_Idle;

parameter *param=new parameter();
ap_t *ap=new ap_t();
AHRS *ahrs=new AHRS(_dt);
EKF_Baro *ekf_baro=new EKF_Baro(_dt, 0.01, 1.0, 0.000016, 0.000016);
EKF_Rangefinder *ekf_rangefinder=new EKF_Rangefinder(_dt, 1.0, 0.000016, 0.16);
EKF_Odometry *ekf_odometry=new EKF_Odometry(_dt, 0.0016, 0.0016, 0.000016, 0.000016, 0.000016, 0.000016);
EKF_GNSS *ekf_gnss=new EKF_GNSS(_dt, 0.0016, 0.0016, 0.0016, 0.0016, 0.000016, 0.000016, 0.000016, 0.000016);
EKF_Wind *ekf_wind=new EKF_Wind(_dt, 0.0016, 0.000016, 0.000016);
Motors *motors=new Motors(1/_dt);
Attitude_Multi *attitude=new Attitude_Multi(*motors, gyro_filt, _dt);
PosControl *pos_control=new PosControl(*motors, *attitude);
CompassCalibrator *compassCalibrator=new CompassCalibrator();
AccelCalibrator *accelCalibrator=new AccelCalibrator();
DataFlash *dataflash=new DataFlash();
UWB *uwb=new UWB();
Rangefinder_state rangefinder_state;
Opticalflow_state opticalflow_state;

float ahrs_pitch_rad(void){return pitch_rad;}
float ahrs_roll_rad(void){return roll_rad;}
float ahrs_yaw_rad(void){return yaw_rad;}
float ahrs_pitch_deg(void){return pitch_deg;}
float ahrs_roll_deg(void){return roll_deg;}
float ahrs_yaw_deg(void){return yaw_deg;}
float ahrs_cos_roll(void){return cos_roll;}
float ahrs_sin_roll(void){return sin_roll;}
float ahrs_cos_pitch(void){return cos_pitch;}
float ahrs_sin_pitch(void){return sin_pitch;}
float ahrs_cos_yaw(void){return cos_yaw;}
float ahrs_sin_yaw(void){return sin_yaw;}
float log_pitch_rad(void){return pitch_log;}
float log_roll_rad(void){return roll_log;}
float log_yaw_rad(void){return yaw_log;}

const Vector3f& get_accel_correct(void){return accel_correct;}	//修正后的三轴机体加速度
const Vector3f& get_gyro_correct(void){return gyro_correct;}	//修正后的三轴机体角速度
const Vector3f& get_mag_correct(void){return mag_correct;}		//修正后的三轴磁场强度
const Vector3f& get_accel_filt(void){return accel_filt;}		//滤波后的三轴机体加速度
const Vector3f& get_gyro_filt(void){return gyro_filt;}			//滤波后的三轴机体角速度
const Vector3f& get_mag_filt(void){return mag_filt;}			//滤波后的三轴磁场强度

const Vector3f& get_accel_ef(void){								//地球坐标系下的三轴加速度m/ss
	return accel_ef;
}

const Vector3f& get_accel_ef_filt(void){						//滤波后的地球坐标系下的三轴加速度m/ss
	return accel_ef_filt;
}

const Vector3f& get_gyro_ef(void){								//地球坐标系下的三轴角速度
	return gyro_ef;
}

const Matrix3f& get_dcm_matrix(void){
	return dcm_matrix;
}

const Matrix3f& get_dcm_matrix_correct(void){
	return dcm_matrix_correct;
}

//return the vib_value
float get_vib_value(void){//震动强度
  return ahrs->vib_value;
}

//return the vib_angle
float get_vib_angle_z(void){//震动向量与大地坐标系z轴的夹角
  return ahrs->vib_angle;
}

void set_enable_odom(bool enable){
	enable_odom=enable;
}

float get_channel_roll_angle(void){return get_channel_roll()*ROLL_PITCH_YAW_INPUT_MAX;}
float get_channel_pitch_angle(void){return get_channel_pitch()*ROLL_PITCH_YAW_INPUT_MAX;}
float get_channel_yaw_angle(void){return get_channel_yaw()*ROLL_PITCH_YAW_INPUT_MAX;}
Location get_gnss_origin_pos(void){return gnss_origin_pos;}
Location get_gnss_current_pos(void){return gnss_current_pos;}
float get_ned_pos_x(void){return ned_current_pos.x;}
float get_ned_pos_y(void){return ned_current_pos.y;}
float get_ned_pos_z(void){return ned_current_pos.z;}
float get_ned_vel_x(void){return ned_current_vel.x;}
float get_ned_vel_y(void){return ned_current_vel.y;}
float get_ned_vel_z(void){return ned_current_vel.z;}
float get_odom_x(void){return odom_3d.x;}
float get_odom_y(void){return odom_3d.y;}
float get_odom_z(void){return odom_3d.z;}
float get_uwb_x(void){return uwb_pos.x;}
float get_uwb_y(void){return uwb_pos.y;}
float get_uwb_z(void){return uwb_pos.z;}
float get_yaw_map(void){return yaw_map;}
bool get_gnss_location_state(void){return get_gnss_location;}
bool get_gcs_connected(void){return gcs_connected;}
bool get_offboard_connected(void){return offboard_connected;}
float get_mav_x_target(void){return mav_x_target;}
float get_mav_y_target(void){return mav_y_target;}
float get_mav_z_target(void){return mav_z_target;}
float get_mav_vx_target(void){return mav_vx_target;}
float get_mav_vy_target(void){return mav_vy_target;}
float get_mav_vz_target(void){return mav_vz_target;}
float get_mav_ax_target(void){return mav_ax_target;}
float get_mav_ay_target(void){return mav_ay_target;}
float get_mav_az_target(void){return mav_az_target;}
float get_mav_ax_roll_target(void){return mav_ax_roll_target;}
float get_mav_ay_pitch_target(void){return mav_ay_pitch_target;}
float get_mav_yaw_target(void){return mav_yaw_target;}
float get_mav_yaw_rate_target(void){return mav_yaw_rate_target;}
bool get_mav_target_state(void){return get_mav_target;}
void reset_mav_target_state(void){get_mav_target=false;}

void reset_dataflash(void){
	dataflash->reset_addr_num_max();
}

void update_dataflash(void){
	uint16_t addr_num_max=dataflash->get_addr_num_max();
	if(addr_num_max>1000){
		dataflash->reset_addr_num_max();
		addr_num_max=dataflash->get_addr_num_max();
	}
	if(addr_num_max<=0){
		dataflash->set_param_float(param->acro_y_expo.num, param->acro_y_expo.value);
		dataflash->set_param_float(param->acro_yaw_p.num, param->acro_yaw_p.value);
		dataflash->set_param_float(param->throttle_midzone.num, param->throttle_midzone.value);
		dataflash->set_param_float(param->pilot_speed_dn.num, param->pilot_speed_dn.value);
		dataflash->set_param_float(param->pilot_speed_up.num, param->pilot_speed_up.value);
		dataflash->set_param_float(param->rangefinder_gain.num, param->rangefinder_gain.value);
		dataflash->set_param_float(param->angle_max.num, param->angle_max.value);
		dataflash->set_param_float(param->pilot_accel_z.num, param->pilot_accel_z.value);
		dataflash->set_param_float(param->pilot_takeoff_alt.num, param->pilot_takeoff_alt.value);
		dataflash->set_param_float(param->spool_up_time.num, param->spool_up_time.value);
		dataflash->set_param_float(param->throttle_filt.num, param->throttle_filt.value);
		dataflash->set_param_vector3f(param->accel_offsets.num, param->accel_offsets.value);
		dataflash->set_param_vector3f(param->accel_diagonals.num, param->accel_diagonals.value);
		dataflash->set_param_vector3f(param->accel_offdiagonals.num, param->accel_offdiagonals.value);
		dataflash->set_param_vector3f(param->mag_offsets.num, param->mag_offsets.value);
		dataflash->set_param_uint16_channel8(param->channel_range.num, param->channel_range.channel);
		dataflash->set_param_float(param->auto_land_speed.num, param->auto_land_speed.value);
		dataflash->set_param_float(param->angle_roll_p.num, param->angle_roll_p.value);
		dataflash->set_param_float(param->angle_pitch_p.num, param->angle_pitch_p.value);
		dataflash->set_param_float(param->angle_yaw_p.num, param->angle_yaw_p.value);
		dataflash->set_param_pid(param->rate_roll_pid.num, param->rate_roll_pid.value_p, param->rate_roll_pid.value_i,
					param->rate_roll_pid.value_d, param->rate_roll_pid.value_imax, param->rate_roll_pid.value_filt_hz);
		dataflash->set_param_pid(param->rate_pitch_pid.num, param->rate_pitch_pid.value_p, param->rate_pitch_pid.value_i,
					param->rate_pitch_pid.value_d, param->rate_pitch_pid.value_imax, param->rate_pitch_pid.value_filt_hz);
		dataflash->set_param_pid(param->rate_yaw_pid.num, param->rate_yaw_pid.value_p, param->rate_yaw_pid.value_i,
					param->rate_yaw_pid.value_d, param->rate_yaw_pid.value_imax, param->rate_yaw_pid.value_filt_hz);
		dataflash->set_param_float(param->pos_z_p.num, param->pos_z_p.value);
		dataflash->set_param_float(param->vel_z_p.num, param->vel_z_p.value);
		dataflash->set_param_pid(param->accel_z_pid.num, param->accel_z_pid.value_p, param->accel_z_pid.value_i,
					param->accel_z_pid.value_d, param->accel_z_pid.value_imax, param->accel_z_pid.value_filt_hz);
		dataflash->set_param_float(param->pos_xy_p.num, param->pos_xy_p.value);
		dataflash->set_param_pid_2d(param->vel_xy_pid.num, param->vel_xy_pid.value_p, param->vel_xy_pid.value_i,
					param->vel_xy_pid.value_d, param->vel_xy_pid.value_imax, param->vel_xy_pid.value_filt_hz,  param->vel_xy_pid.value_filt_d_hz);
		dataflash->set_param_uint8(param->robot_type.num, param->robot_type.value);
		dataflash->set_param_uint8(param->motor_type.num, param->motor_type.value);
		dataflash->set_param_float(param->t_hover_update_min.num, param->t_hover_update_min.value);
		dataflash->set_param_float(param->t_hover_update_max.num, param->t_hover_update_max.value);
		dataflash->set_param_float(param->vib_land.num, param->vib_land.value);
		dataflash->set_param_vector3f(param->horizontal_correct.num, param->horizontal_correct.value);
		dataflash->set_param_float(param->lowbatt_return_volt.num, param->lowbatt_return_volt.value);
		dataflash->set_param_float(param->lowbatt_land_volt.num, param->lowbatt_land_volt.value);
		dataflash->set_param_float(param->poshold_vel_max.num, param->poshold_vel_max.value);
		dataflash->set_param_float(param->poshold_accel_max.num, param->poshold_accel_max.value);
		dataflash->set_param_float(param->mission_vel_max.num, param->mission_vel_max.value);
		dataflash->set_param_float(param->mission_accel_max.num, param->mission_accel_max.value);
		dataflash->set_param_float(param->alt_return.num, param->alt_return.value);
		dataflash->set_param_float(param->voltage_gain.num, param->voltage_gain.value);
		dataflash->set_param_float(param->current_gain.num, param->current_gain.value);
		dataflash->set_param_float(param->uwb_yaw_delta_deg.num, param->uwb_yaw_delta_deg.value);
		dataflash->set_param_uint8(param->uwb_tag_id.num, param->uwb_tag_id.value);
		dataflash->set_param_uint8(param->uwb_tag_max.num, param->uwb_tag_max.value);
		dataflash->set_param_vector3f(param->uwb_anchor01_pos.num, param->uwb_anchor01_pos.value);
		dataflash->set_param_vector3f(param->uwb_anchor02_pos.num, param->uwb_anchor02_pos.value);
		dataflash->set_param_vector3f(param->uwb_anchor03_pos.num, param->uwb_anchor03_pos.value);
		dataflash->set_param_vector3f(param->uwb_anchor04_pos.num, param->uwb_anchor04_pos.value);
		dataflash->set_param_float(param->auto_takeoff_speed.num, param->auto_takeoff_speed.value);
		dataflash->set_param_float(param->baro_temp_offset_gain.num, param->baro_temp_offset_gain.value);
		dataflash->set_param_float(param->landing_lock_alt.num, param->landing_lock_alt.value);
		dataflash->set_param_vector3f(param->flow_gain.num, param->flow_gain.value);
		dataflash->set_param_vector3f(param->gnss_offset.num, param->gnss_offset.value);
		dataflash->set_param_vector3f(param->mag_diagonals.num, param->mag_diagonals.value);
		dataflash->set_param_vector3f(param->mag_offdiagonals.num, param->mag_offdiagonals.value);
		dataflash->set_param_uint32(param->comm1_bandrate.num, param->comm1_bandrate.value);
		dataflash->set_param_uint32(param->comm2_bandrate.num, param->comm2_bandrate.value);
		dataflash->set_param_uint32(param->comm3_bandrate.num, param->comm3_bandrate.value);
		dataflash->set_param_uint32(param->comm4_bandrate.num, param->comm4_bandrate.value);
		/* *************************************************
		* ****************Dev code begin*******************/
		// Warning! Developer can add your new code here!
		/* Demo
		 此处添加您的自定义参数, e.g.
		 dataflash->set_param_vector3f(param->demo_param_1.num, param->demo_param_1.value);
		 dataflash->set_param_float(param->demo_param_2.num, param->demo_param_2.value);
		 * */

		/* ****************Dev code end*********************
		* *************************************************/
	}else{
		dataflash->get_param_float(param->acro_y_expo.num, param->acro_y_expo.value);
		dataflash->get_param_float(param->acro_yaw_p.num, param->acro_yaw_p.value);
		dataflash->get_param_float(param->throttle_midzone.num, param->throttle_midzone.value);
		dataflash->get_param_float(param->pilot_speed_dn.num, param->pilot_speed_dn.value);
		dataflash->get_param_float(param->pilot_speed_up.num, param->pilot_speed_up.value);
		dataflash->get_param_float(param->rangefinder_gain.num, param->rangefinder_gain.value);
		dataflash->get_param_float(param->angle_max.num, param->angle_max.value);
		dataflash->get_param_float(param->pilot_accel_z.num, param->pilot_accel_z.value);
		dataflash->get_param_float(param->pilot_takeoff_alt.num, param->pilot_takeoff_alt.value);
		dataflash->get_param_float(param->spool_up_time.num, param->spool_up_time.value);
		dataflash->get_param_float(param->throttle_filt.num, param->throttle_filt.value);
		dataflash->get_param_vector3f(param->accel_offsets.num, param->accel_offsets.value);
		dataflash->get_param_vector3f(param->accel_diagonals.num, param->accel_diagonals.value);
		dataflash->get_param_vector3f(param->accel_offdiagonals.num, param->accel_offdiagonals.value);
		dataflash->get_param_vector3f(param->mag_offsets.num, param->mag_offsets.value);
		dataflash->get_param_uint16_channel8(param->channel_range.num, param->channel_range.channel);
		dataflash->get_param_float(param->auto_land_speed.num, param->auto_land_speed.value);
		dataflash->get_param_float(param->angle_roll_p.num, param->angle_roll_p.value);
		dataflash->get_param_float(param->angle_pitch_p.num, param->angle_pitch_p.value);
		dataflash->get_param_float(param->angle_yaw_p.num, param->angle_yaw_p.value);
		dataflash->get_param_pid(param->rate_roll_pid.num, param->rate_roll_pid.value_p, param->rate_roll_pid.value_i,
					param->rate_roll_pid.value_d, param->rate_roll_pid.value_imax, param->rate_roll_pid.value_filt_hz);
		dataflash->get_param_pid(param->rate_pitch_pid.num, param->rate_pitch_pid.value_p, param->rate_pitch_pid.value_i,
					param->rate_pitch_pid.value_d, param->rate_pitch_pid.value_imax, param->rate_pitch_pid.value_filt_hz);
		dataflash->get_param_pid(param->rate_yaw_pid.num, param->rate_yaw_pid.value_p, param->rate_yaw_pid.value_i,
					param->rate_yaw_pid.value_d, param->rate_yaw_pid.value_imax, param->rate_yaw_pid.value_filt_hz);
		dataflash->get_param_float(param->pos_z_p.num, param->pos_z_p.value);
		dataflash->get_param_float(param->vel_z_p.num, param->vel_z_p.value);
		dataflash->get_param_pid(param->accel_z_pid.num, param->accel_z_pid.value_p, param->accel_z_pid.value_i,
					param->accel_z_pid.value_d, param->accel_z_pid.value_imax, param->accel_z_pid.value_filt_hz);
		dataflash->get_param_float(param->pos_xy_p.num, param->pos_xy_p.value);
		dataflash->get_param_pid_2d(param->vel_xy_pid.num, param->vel_xy_pid.value_p, param->vel_xy_pid.value_i,
					param->vel_xy_pid.value_d, param->vel_xy_pid.value_imax, param->vel_xy_pid.value_filt_hz, param->vel_xy_pid.value_filt_d_hz);
		dataflash->get_param_uint8(param->robot_type.num, param->robot_type.value);
		dataflash->get_param_uint8(param->motor_type.num, param->motor_type.value);
		dataflash->get_param_float(param->t_hover_update_min.num, param->t_hover_update_min.value);
		dataflash->get_param_float(param->t_hover_update_max.num, param->t_hover_update_max.value);
		dataflash->get_param_float(param->vib_land.num, param->vib_land.value);
		dataflash->get_param_vector3f(param->horizontal_correct.num, param->horizontal_correct.value);
		dataflash->get_param_float(param->lowbatt_return_volt.num, param->lowbatt_return_volt.value);
		dataflash->get_param_float(param->lowbatt_land_volt.num, param->lowbatt_land_volt.value);
		dataflash->get_param_float(param->poshold_vel_max.num, param->poshold_vel_max.value);
		dataflash->get_param_float(param->poshold_accel_max.num, param->poshold_accel_max.value);
		dataflash->get_param_float(param->mission_vel_max.num, param->mission_vel_max.value);
		dataflash->get_param_float(param->mission_accel_max.num, param->mission_accel_max.value);
		dataflash->get_param_float(param->alt_return.num, param->alt_return.value);
		dataflash->get_param_float(param->voltage_gain.num, param->voltage_gain.value);
		dataflash->get_param_float(param->current_gain.num, param->current_gain.value);
		dataflash->get_param_float(param->uwb_yaw_delta_deg.num, param->uwb_yaw_delta_deg.value);
		dataflash->get_param_uint8(param->uwb_tag_id.num, param->uwb_tag_id.value);
		dataflash->get_param_uint8(param->uwb_tag_max.num, param->uwb_tag_max.value);
		dataflash->get_param_vector3f(param->uwb_anchor01_pos.num, param->uwb_anchor01_pos.value);
		dataflash->get_param_vector3f(param->uwb_anchor02_pos.num, param->uwb_anchor02_pos.value);
		dataflash->get_param_vector3f(param->uwb_anchor03_pos.num, param->uwb_anchor03_pos.value);
		dataflash->get_param_vector3f(param->uwb_anchor04_pos.num, param->uwb_anchor04_pos.value);
		dataflash->get_param_float(param->auto_takeoff_speed.num, param->auto_takeoff_speed.value);
		dataflash->get_param_float(param->baro_temp_offset_gain.num, param->baro_temp_offset_gain.value);
		dataflash->get_param_float(param->landing_lock_alt.num, param->landing_lock_alt.value);
		dataflash->get_param_vector3f(param->flow_gain.num, param->flow_gain.value);
		dataflash->get_param_vector3f(param->gnss_offset.num, param->gnss_offset.value);
		dataflash->get_param_vector3f(param->mag_diagonals.num, param->mag_diagonals.value);
		dataflash->get_param_vector3f(param->mag_offdiagonals.num, param->mag_offdiagonals.value);
		dataflash->get_param_uint32(param->comm1_bandrate.num, param->comm1_bandrate.value);
		dataflash->get_param_uint32(param->comm2_bandrate.num, param->comm2_bandrate.value);
		dataflash->get_param_uint32(param->comm3_bandrate.num, param->comm3_bandrate.value);
		dataflash->get_param_uint32(param->comm4_bandrate.num, param->comm4_bandrate.value);

		/* *************************************************
		 * ****************Dev code begin*******************/
		// Warning! Developer can add your new code here!
		/* Demo
		 此处添加您的自定义参数, e.g.
		 dataflash->get_param_vector3f(param->demo_param_1.num, param->demo_param_1.value);
		 dataflash->get_param_float(param->demo_param_2.num, param->demo_param_2.value);
		 * */

		/* ****************Dev code end*********************
		 * *************************************************/
	}
}

void set_comm_bandrate(void){
	set_s1_baudrate(param->comm1_bandrate.value);
	set_s2_baudrate(param->comm2_bandrate.value);
	set_s3_baudrate(param->comm3_bandrate.value);
	set_s4_baudrate(param->comm4_bandrate.value);
}

#define TFMINI_DATA_Len             9
#define TFMINT_DATA_HEAD            0x59
typedef enum{
	TFMINI_IDLE=0,
	TFMINI_HEAD,
	TFMINI_CHK
}TFmini_state;
static TFmini_state tfmini_state=TFMINI_IDLE;
static uint8_t chk_cal = 0, data_num=0;
static uint8_t tfmini_data[6];
static Vector3f tfmini_offset=Vector3f(0.0f, 0.0f, 7.0f);//激光测距仪相对于机体中心的坐标,单位:cm (机头方向为x轴正方向, 机体右侧为y轴正方向)
static uint16_t cordist = 0, strength=0;
static bool use_tfmini=false, force_vl53lxx=false;
void get_tfmini_data(uint8_t buf)
{
	if(!use_rangefinder){
		rangefinder_state.alt_healthy=false;
		return;
	}
	if(USE_ODOM_Z){
		return;
	}
	if(force_vl53lxx){
		return;
	}
	switch(tfmini_state){
		case TFMINI_IDLE:
			if(TFMINT_DATA_HEAD==buf){
				tfmini_state=TFMINI_HEAD;
				chk_cal=TFMINT_DATA_HEAD;
				data_num=0;
			}
			break;
		case TFMINI_HEAD:
			if(TFMINT_DATA_HEAD==buf){
				chk_cal+=TFMINT_DATA_HEAD;
				tfmini_state=TFMINI_CHK;
			}else{
				tfmini_state=TFMINI_IDLE;
			}
			break;
		case TFMINI_CHK:
			if(data_num<6){
				chk_cal+=buf;
				tfmini_data[data_num]=buf;
				data_num++;
			}else if(data_num==6&&chk_cal==buf){
				cordist=tfmini_data[0]|(tfmini_data[1]<<8);//cm
				strength=tfmini_data[2]|(tfmini_data[3]<<8);
				rangefinder_state.last_update_ms=HAL_GetTick();
				if(cordist>3&&cordist<=800){
					use_tfmini=true;
					Vector3f pos_offset=dcm_matrix*tfmini_offset;
					if(!rangefinder_state.alt_healthy){
						rangefinder_state.alt_cm_filt.reset((float)cordist);//重置滤波器
					}
					rangefinder_state.alt_cm=rangefinder_state.alt_cm_filt.apply((float)cordist, (float)(HAL_GetTick()-rangefinder_state.last_healthy_ms)*0.001);
					rangefinder_state.alt_cm=rangefinder_state.alt_cm*MAX(0.707f, dcm_matrix.c.z)+pos_offset.z;
					rangefinder_state.last_healthy_ms=HAL_GetTick();
					get_rangefinder_data=true;
					enable_surface_track=true;
					if(cordist>100){
						rangefinder_state.enabled=true;
					}
//					usb_printf("dis1:%f\n",rangefinder_state.alt_cm);
					rangefinder_state.alt_healthy=true;
				}else{
					if(cordist<=3){
						use_tfmini=false;
					}
					rangefinder_state.alt_healthy=false;
				}
				tfmini_state=TFMINI_IDLE;
			}else{
				tfmini_state=TFMINI_IDLE;
			}
			break;
	}
}

static Vector3f vl53lxx_offset=Vector3f(0.0f, 0.0f, 0.0f);//激光测距仪相对于机体中心的坐标,单位:cm (机头方向为x轴正方向, 机体右侧为y轴正方向)
void get_vl53lxx_data(uint16_t distance_mm){
	if(!use_rangefinder){
		rangefinder_state.alt_healthy=false;
		return;
	}
	if(USE_ODOM_Z){
		return;
	}
	if(distance_mm>5&&distance_mm<100){
		force_vl53lxx=true;
	}else{
		force_vl53lxx=false;
		if(use_tfmini){
			return;
		}
	}
	rangefinder_state.last_update_ms=HAL_GetTick();
	if(get_gnss_state()){
		enable_surface_track=false;
	}else{
		enable_surface_track=true;
	}
	if(distance_mm>0&&distance_mm<4000){//4m以下数据才可靠
		Vector3f pos_offset=dcm_matrix*vl53lxx_offset;
		if(!rangefinder_state.alt_healthy){
			rangefinder_state.alt_cm_filt.reset((float)distance_mm*0.1);//重置滤波器
		}
		rangefinder_state.alt_cm=rangefinder_state.alt_cm_filt.apply((float)distance_mm*0.1, (float)(HAL_GetTick()-rangefinder_state.last_healthy_ms)*0.001);
		rangefinder_state.alt_cm=rangefinder_state.alt_cm*MAX(0.707f, dcm_matrix.c.z)+pos_offset.z;
		rangefinder_state.last_healthy_ms=HAL_GetTick();
		get_rangefinder_data=true;
		if(distance_mm>1000){
			rangefinder_state.enabled=true;
		}
//		usb_printf("dis2:%d\n",distance_mm);
		rangefinder_state.alt_healthy=true;
	}else{
		rangefinder_state.alt_healthy=false;
	}
}

static Vector3f tf2mini_offset=Vector3f(0.0f, 0.0f, 0.0f);//激光测距仪相对于机体中心的坐标,单位:cm (机头方向为x轴正方向, 机体右侧为y轴正方向)
void update_tf2mini_data(float dis){
	if(!use_rangefinder){
		rangefinder_state.alt_healthy=false;
		return;
	}
	if(USE_ODOM_Z){
		return;
	}
	if(force_vl53lxx){
		return;
	}
	rangefinder_state.last_update_ms=HAL_GetTick();
	if(dis>3.0f&&dis<=750.0f){
		use_tfmini=true;
		Vector3f pos_offset=dcm_matrix*tf2mini_offset;
		if(!rangefinder_state.alt_healthy){
			rangefinder_state.alt_cm_filt.reset(dis);//重置滤波器
		}
		rangefinder_state.alt_cm=rangefinder_state.alt_cm_filt.apply(dis, (float)(HAL_GetTick()-rangefinder_state.last_healthy_ms)*0.001);
		rangefinder_state.alt_cm=rangefinder_state.alt_cm*MAX(0.707f, dcm_matrix.c.z)+pos_offset.z;
		rangefinder_state.last_healthy_ms=HAL_GetTick();
		get_rangefinder_data=true;
		enable_surface_track=true;
		if(dis>100.0f){
			rangefinder_state.enabled=true;
		}
//		usb_printf("dis3:%f\n",rangefinder_state.alt_cm);
		rangefinder_state.alt_healthy=true;
	}else{
		if(dis<=3.0f){
			use_tfmini=false;
		}
		rangefinder_state.alt_healthy=false;
	}
}

#define flow_sample_num 5
static float flow_alt, flow_bf_x, flow_bf_y;
static Vector3f flow_gyro_offset;
static Vector2f flow_vel, flow_vel_delta, flow_vel_sample[flow_sample_num];
static float flow_gain_x=-0.025, flow_gain_y=0.025, flow_gain_z=0.0025;
static uint8_t flow_sample_flag=0,flow_i_buff=0;
static bool lose_flow=false;
static float ned_pos_x_buff[50], ned_pos_y_buff[50];
void opticalflow_update(void){
#if USE_FLOW
	if(rangefinder_state.alt_healthy){
		flow_alt=rangefinder_state.alt_cm;
	}else{
		flow_alt=get_pos_z()-takeoff_alt;//cm
	}
	flow_alt=constrain_float(flow_alt, 3.0f, 150.0f);
	if(lc302_data.quality==245){
		opticalflow_state.healthy=true;
		get_opticalflow=true;
	}else{
		opticalflow_state.healthy=false;
		flow_sample_flag=0;
		return;
	}
	//光流坐标系->机体坐标系//TODO:add gyro offset
	flow_bf_x=(float)lc302_data.flow_y_integral*0.0001f+constrain_float(flow_gyro_offset.y*flow_gain_x*param->flow_gain.value.x, -0.2, 0.2);
	flow_bf_y=-(float)lc302_data.flow_x_integral*0.0001f+constrain_float(flow_gyro_offset.x*flow_gain_y*param->flow_gain.value.y, -0.2, 0.2)+constrain_float(flow_gain_z*flow_gyro_offset.z*param->flow_gain.value.z, -0.2, 0.2);
//	usb_printf_dir("$%d %d;", lc302_data.flow_x_integral, (int16_t)(constrain_float(flow_gain_z*flow_gyro_offset.z*param->flow_gain.value.z, -0.2, 0.2)*10000));
	//机体坐标系->大地坐标系
	opticalflow_state.rads.x=flow_bf_x*ahrs_cos_yaw()-flow_bf_y*ahrs_sin_yaw();
	opticalflow_state.rads.y=flow_bf_x*ahrs_sin_yaw()+flow_bf_y*ahrs_cos_yaw();
	opticalflow_state.flow_dt=(float)lc302_data.integration_timespan*0.000001f;
	flow_vel_delta.x=MAX(get_accel_ef().x*100*0.3,30.0f);
	flow_vel_delta.y=MAX(get_accel_ef().y*100*0.3,30.0f);
	flow_vel.x=constrain_float(opticalflow_state.rads.x*flow_alt/opticalflow_state.flow_dt, opticalflow_state.vel.x-flow_vel_delta.x, opticalflow_state.vel.x+flow_vel_delta.x);
	flow_vel.y=constrain_float(opticalflow_state.rads.y*flow_alt/opticalflow_state.flow_dt, opticalflow_state.vel.y-flow_vel_delta.y, opticalflow_state.vel.y+flow_vel_delta.y);
	if(abs(flow_vel.length()-ekf_baro->vel_2d)>20.0f&&!lose_flow){//奇异值
		lose_flow=true;
	}else{
		if(flow_sample_flag<flow_sample_num){
			flow_vel_sample[flow_sample_flag]=flow_vel;
		}else{
			flow_vel_sample[flow_sample_flag%flow_sample_num]=flow_vel;
			flow_vel.zero();
			for(uint8_t i=0;i<flow_sample_num;i++){
				flow_vel+=flow_vel_sample[i];
			}
			flow_vel*=1.0/flow_sample_num;
		}
		flow_sample_flag++;
		if(flow_sample_flag==flow_sample_num*2){
			flow_sample_flag=flow_sample_num;
		}
		opticalflow_state.vel=flow_vel;
		lose_flow=false;
	}
	opticalflow_state.pos+=opticalflow_state.vel*opticalflow_state.flow_dt;
//	usb_printf_dir("$%d %d;",(int16_t)flow_vel.y, (int16_t)flow_vel_sample[flow_sample_flag%10].y);
	if(!get_gnss_state()||!USE_MAG){
		get_gnss_location=true;
		ned_current_vel.x=opticalflow_state.vel.x;
		ned_current_vel.y=opticalflow_state.vel.y;
		ned_current_pos.x+=opticalflow_state.vel.x*opticalflow_state.flow_dt;
		ned_current_pos.y+=opticalflow_state.vel.y*opticalflow_state.flow_dt;
		ned_pos_x_buff[flow_i_buff]=opticalflow_state.pos.x;
		ned_pos_y_buff[flow_i_buff]=opticalflow_state.pos.y;
		flow_i_buff++;
		if(flow_i_buff==50){
			flow_i_buff=0;
		}
	}
#endif
}

//接收
static uint32_t time_last_heartbeat[MAVLINK_COMM_NUM_BUFFERS]={0};
static uint32_t time_last_attitude=0, get_odom_time=0;
static mavlink_heartbeat_t heartbeat;
static mavlink_set_mode_t setmode;
static mavlink_mission_count_t mission_count;
static mavlink_mission_item_t mission_item;
static mavlink_log_request_data_t log_request_data;
static mavlink_command_ack_t ack;
static mavlink_command_long_t cmd;
static mavlink_rc_channels_override_t rc_channels;
static mavlink_attitude_t attitude_mav;
static mavlink_local_position_ned_cov_t local_position_ned_cov;
static mavlink_set_position_target_local_ned_t set_position_target_local_ned, set_goal_point;
static mavlink_global_vision_position_estimate_t pose;
static Vector3f lidar_offset=Vector3f(0.0f,0.0f, 0.0f);//cm
static uint8_t gcs_channel=255,offboard_channel=255;
static uint16_t gnss_point_statis=0;
static uint8_t gnss_reset_notify=0;
static float motor_test_type=0.0f,motor_test_throttle=0.0f,motor_test_timeout=0.0f,motor_test_num=0.0f;
static uint32_t motor_test_start_time=0;
static float odom_2d=0.0f,odom_2d_x_last=0.0f,odom_2d_y_last=0.0f, odom_dx=0.0f, odom_dy=0.0f, odom_dz=0.0f, odomz_tc=0.0f, odomz_dt=0.0f;
uint8_t get_coordinate_mode(void){
	return set_position_target_local_ned.coordinate_frame;
}

uint8_t get_gnss_reset_notify(void){
	return gnss_reset_notify;
}

bool get_force_autonav(void){
	return force_autonav;
}

//发送
static mavlink_message_t msg_heartbeat, msg_set_goal_point;
static mavlink_heartbeat_t heartbeat_send;
static mavlink_system_t mavlink_system;
static mavlink_message_t msg_global_attitude_position, msg_global_position_int, msg_command_long, msg_battery_status, msg_rc_channels, msg_mission_count, msg_mission_item, msg_system_version;
static mavlink_global_vision_position_estimate_t global_attitude_position;
static mavlink_global_position_int_t global_position_int;
static mavlink_mission_count_t mission_count_send;
static mavlink_mission_item_t mission_item_send;
static mavlink_command_long_t command_long;
static mavlink_battery_status_t battery_status;
static mavlink_rc_channels_t rc_channels_t;
static mavlink_timesync_t system_version;

void parse_mavlink_data(mavlink_channel_t chan, uint8_t data, mavlink_message_t* msg_received, mavlink_status_t* status){
	if (mavlink_parse_char(chan, data, msg_received, status)){
		switch (msg_received->msgid) {
			case MAVLINK_MSG_ID_HEARTBEAT:
				mavlink_msg_heartbeat_decode(msg_received, &heartbeat);
				if((HeartBeatFlags&(EVENTBIT_HEARTBEAT_COMM_0<<(uint8_t)chan))==0){
					HeartBeatFlags|=(EVENTBIT_HEARTBEAT_COMM_0<<(uint8_t)chan);
					system_version.tc1=VERSION_HARDWARE;
					system_version.ts1=VERSION_FIRMWARE;
					mavlink_msg_timesync_encode(mavlink_system.sysid, mavlink_system.compid, &msg_system_version, &system_version);
					mavlink_send_buffer(chan, &msg_system_version);
				}
				time_last_heartbeat[(uint8_t)chan]=HAL_GetTick();
				if(heartbeat.type==MAV_TYPE_GCS){//地面站
					if(!gcs_connected){
						gcs_channel=chan;
						gcs_connected=true;
						if(!get_soft_armed()){
							Buzzer_set_ring_type(BUZZER_MAV_CONNECT);
						}
					}
					if(chan==MAVLINK_COMM_0){
						offboard_connected=true;
						offboard_channel=MAVLINK_COMM_0;
					}
				}else if(heartbeat.type==MAV_TYPE_ONBOARD_CONTROLLER){//机载电脑
					if(!offboard_connected){
						offboard_connected=true;
						offboard_channel=chan;
						if(!get_soft_armed()){
							Buzzer_set_ring_type(BUZZER_MAV_CONNECT);
						}
					}
				}else if(heartbeat.type==MAV_TYPE_CAMERA){
					if(!offboard_connected){
						offboard_channel=chan;
						if(!get_soft_armed()){
							Buzzer_set_ring_type(BUZZER_MAV_CONNECT);
						}
					}
				}
				if(msg_received->sysid==254){
					force_autonav=true;
				}else{
					force_autonav=false;
				}
				break;
			case MAVLINK_MSG_ID_SET_MODE:
				mavlink_msg_set_mode_decode(msg_received, &setmode);
				if(setmode.base_mode==MAV_MODE_AUTO_ARMED){
					unlock_motors();
					break;
				}else if(setmode.base_mode==MAV_MODE_AUTO_DISARMED){
					disarm_motors();
					lock_motors();
					break;
				}else if(setmode.base_mode==MAV_MODE_PREFLIGHT){
					if(get_soft_armed()||motors->get_interlock()||motors->get_armed()){
						break;
					}
					param->robot_type.value=(uint8_t)((setmode.custom_mode>>24)&0xFF);
					param->motor_type.value=(uint8_t)((setmode.custom_mode>>16)&0xFF);
					dataflash->set_param_uint8(param->robot_type.num, param->robot_type.value);
					dataflash->set_param_uint8(param->motor_type.num, param->motor_type.value);
					motors_init();
				}
				break;
			case MAVLINK_MSG_ID_MISSION_COUNT:
				mavlink_msg_mission_count_decode(msg_received, &mission_count);
				send_mavlink_mission_ack(chan, MAV_MISSION_ACCEPTED);
				gnss_point_statis=0;
				gnss_reset_notify++;
				break;
			case MAVLINK_MSG_ID_MISSION_ITEM:
				mavlink_msg_mission_item_decode(msg_received, &mission_item);
				if(mission_item.seq==gnss_point_statis){
					gnss_point_statis++;//统计收到的航点数
				}
				//seq表示当前航点序号,x:lat,y:lon,z:alt
				gnss_point_prt[mission_item.seq].x=mission_item.x;
				gnss_point_prt[mission_item.seq].y=mission_item.y;
				gnss_point_prt[mission_item.seq].z=mission_item.z;
				send_mavlink_mission_item_reached(chan, mission_item.seq);
				if(mission_count.count==mission_item.seq+1){//最后一个点接收完要把航点写入内存卡
					if(gnss_point_statis==mission_count.count){//如果统计航点数=航点总数，表示全部航点已被接收
						//航点总数
						gnss_point_num=mission_count.count;
						Logger_Write_Gnss();
					}
					gnss_point_statis=0;
				}
				break;
			case MAVLINK_MSG_ID_MISSION_REQUEST:
				//地面站请求航点总数
				send_mavlink_mission_count(chan);
				break;
			case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
				//地面站请求航点列表
				send_mavlink_mission_list(chan);
				break;
			case MAVLINK_MSG_ID_LOG_ENTRY:
				if(m_Logger_Status==Logger_Idle){
					sd_get_file_name(chan);
				}
				break;
			case MAVLINK_MSG_ID_LOG_REQUEST_DATA:
				mavlink_msg_log_request_data_decode(msg_received, &log_request_data);
				if(m_Logger_Status==Logger_Idle){
					sd_send_log_file(chan, log_request_data.id-1);
				}
				break;
			case MAVLINK_MSG_ID_COMMAND_ACK:
				mavlink_msg_command_ack_decode(msg_received, &ack);
				if(ack.command==MAV_CMD_CONDITION_DISTANCE){
					if(ack.result>0&&ack.result<=3){
						uwb->config_uwb((uwb_modes)ack.result, param->uwb_tag_id.value, 1, 1, param->uwb_tag_max.value, 4);
					}
				}
				break;
			case MAVLINK_MSG_ID_COMMAND_LONG:
				mavlink_msg_command_long_decode(msg_received, &cmd);
				switch(cmd.command){
					case MAV_CMD_NAV_TAKEOFF:
						set_takeoff();
						send_mavlink_commond_ack(chan, MAV_CMD_NAV_TAKEOFF, MAV_CMD_ACK_OK);
						break;
					case MAV_CMD_NAV_RETURN_TO_LAUNCH:
						set_return(true);
						send_mavlink_commond_ack(chan, MAV_CMD_NAV_RETURN_TO_LAUNCH, MAV_CMD_ACK_OK);
						break;
					case MAV_CMD_NAV_LAND:
						if(robot_state==STATE_FLYING){
							robot_state_desired=STATE_LANDED;
							send_mavlink_commond_ack(chan, MAV_CMD_NAV_LAND, MAV_CMD_ACK_OK);
						}
						break;
					case MAV_CMD_MISSION_START:
						gnss_reset_notify++;
						send_mavlink_commond_ack(chan, MAV_CMD_MISSION_START, MAV_CMD_ACK_OK);
						break;
					case MAV_CMD_DO_MOTOR_TEST:
						motor_test_type=cmd.param1; 	//1.0
						motor_test_throttle=cmd.param2; //0~1.0
						motor_test_timeout=cmd.param3; 	//单位：s
						motor_test_num=cmd.param4;		//0~8
						motor_test_start_time=HAL_GetTick();
						break;
					case MAV_CMD_PREFLIGHT_CALIBRATION:
						if(is_equal(cmd.param1,1.0f)){					//start accel calibrate
							if(is_equal(cmd.param2,0.0f)){
								accel_cal_succeed=false;
								param->accel_offsets.value={0.0f,0.0f,0.0f};
								param->accel_diagonals.value={1.0f,1.0f,1.0f};
								param->accel_offdiagonals.value={0.0f,0.0f,0.0f};
							}else {
								accelCalibrator->collect_sample();
							}
						}else if(is_equal(cmd.param1,2.0f)){			//start compass calibrate
							compass_cal_succeed=false;
							initial_compass_cal=false;
							mag_correcting=true;
							mag_corrected=false;
							ahrs->reset();
							param->mag_offsets.value={0.0f,0.0f,0.0f};
							param->mag_diagonals.value={1.0f,1.0f,1.0f};
							param->mag_offdiagonals.value={0.0f,0.0f,0.0f};
						}else if(is_equal(cmd.param1,3.0f)){            //校准水平
							horizon_correct=true;
							reset_horizon_integrator=true;
							param->vel_pid_integrator.value={0.0f,0.0f,0.0f};
							param->rate_pid_integrator.value={0.0f,0.0f,0.0f};
						}
						send_mavlink_commond_ack(chan, MAV_CMD_PREFLIGHT_CALIBRATION, MAV_CMD_ACK_OK);
						break;
					case MAV_CMD_DO_SET_PARAMETER:
						switch(uint16_t(cmd.param1)){
						case 0://reset PID parameters in flash

							/* *************************************************
							 * ****************Dev code begin*******************/
							// Warning! Developer can add your new code here!
							/* Demo
							 如果希望通过app的一键重置按钮把参数重置为默认值，那么把参数重置代码添加在这里
							 param->demo_param_1.value={1.0,1.0,1.0};
							 dataflash->set_param_float(param->demo_param_1.num, param->demo_param_1.value);
							 param->demo_param_2.value=1.0f;
							 dataflash->set_param_float(param->demo_param_2.num, param->demo_param_2.value);
							 * */

							/* ****************Dev code end*********************
							 * *************************************************/

							param->angle_roll_p.value=AC_ATTITUDE_CONTROL_ANGLE_ROLL_P;
							param->angle_pitch_p.value=AC_ATTITUDE_CONTROL_ANGLE_PITCH_P;
							param->angle_yaw_p.value=AC_ATTITUDE_CONTROL_ANGLE_YAW_P;
							param->rate_roll_pid.value_p=AC_ATC_MULTI_RATE_ROLL_P;
							param->rate_roll_pid.value_i=AC_ATC_MULTI_RATE_ROLL_I;
							param->rate_roll_pid.value_d=AC_ATC_MULTI_RATE_ROLL_D;
							param->rate_roll_pid.value_imax=AC_ATC_MULTI_RATE_RP_IMAX;
							param->rate_roll_pid.value_filt_hz=AC_ATC_MULTI_RATE_RP_FILT_HZ;
							param->rate_pitch_pid.value_p=AC_ATC_MULTI_RATE_PITCH_P;
							param->rate_pitch_pid.value_i=AC_ATC_MULTI_RATE_PITCH_I;
							param->rate_pitch_pid.value_d=AC_ATC_MULTI_RATE_PITCH_D;
							param->rate_pitch_pid.value_imax=AC_ATC_MULTI_RATE_RP_IMAX;
							param->rate_pitch_pid.value_filt_hz=AC_ATC_MULTI_RATE_RP_FILT_HZ;
							param->rate_yaw_pid.value_p=AC_ATC_MULTI_RATE_YAW_P;
							param->rate_yaw_pid.value_i=AC_ATC_MULTI_RATE_YAW_I;
							param->rate_yaw_pid.value_d=AC_ATC_MULTI_RATE_YAW_D;
							param->rate_yaw_pid.value_imax=AC_ATC_MULTI_RATE_YAW_IMAX;
							param->rate_yaw_pid.value_filt_hz=AC_ATC_MULTI_RATE_YAW_FILT_HZ;
							param->pos_z_p.value=POSCONTROL_POS_Z_P;
							param->vel_z_p.value=POSCONTROL_VEL_Z_P;
							param->accel_z_pid.value_p=POSCONTROL_ACC_Z_P;
							param->accel_z_pid.value_i=POSCONTROL_ACC_Z_I;
							param->accel_z_pid.value_d=POSCONTROL_ACC_Z_D;
							param->accel_z_pid.value_imax=POSCONTROL_ACC_Z_IMAX;
							param->accel_z_pid.value_filt_hz=POSCONTROL_ACC_Z_FILT_HZ;
							param->pos_xy_p.value=POSCONTROL_POS_XY_P;
							param->vel_xy_pid.value_p=POSCONTROL_VEL_XY_P;
							param->vel_xy_pid.value_i=POSCONTROL_VEL_XY_I;
							param->vel_xy_pid.value_d=POSCONTROL_VEL_XY_D;
							param->vel_xy_pid.value_imax=POSCONTROL_VEL_XY_IMAX;
							param->vel_xy_pid.value_filt_hz=POSCONTROL_VEL_XY_FILT_HZ;
							param->vel_xy_pid.value_filt_d_hz=POSCONTROL_VEL_XY_FILT_D_HZ;

							param->acro_y_expo.value=ACRO_YAW_EXPO;
							param->acro_yaw_p.value=ACRO_YAW_P;
							param->throttle_midzone.value=THR_MIDZ_DEFAULT;
							param->pilot_speed_dn.value=PILOT_VELZ_DOWN_MAX;
							param->pilot_speed_up.value=PILOT_VELZ_UP_MAX;
							param->auto_land_speed.value=AUTO_LAND_SPEED;
							param->rangefinder_gain.value=RANGEFINDER_GAIN_DEFAULT;
							param->angle_max.value=DEFAULT_ANGLE_MAX;
							param->pilot_accel_z.value=PILOT_ACCEL_Z_DEFAULT;
							param->pilot_takeoff_alt.value=PILOT_TKOFF_ALT_DEFAULT;
							param->spool_up_time.value=MOTORS_SPOOL_UP_TIME_DEFAULT;
							param->throttle_filt.value=MAN_THR_FILT_HZ;
							param->t_hover_update_min.value=THR_HOVER_UPDATE_MIN;
							param->t_hover_update_max.value=THR_HOVER_UPDATE_MAX;
							param->vib_land.value=VIB_LAND_THR;
							param->lowbatt_return_volt.value=LOWBATT_RETURN_VOLT;
							param->lowbatt_land_volt.value=LOWBATT_LAND_VOLT;
							param->poshold_vel_max.value=POSHOLD_VEL_MAX;
							param->poshold_accel_max.value=POSHOLD_ACCEL_MAX;
							param->mission_vel_max.value=MISSION_VEL_MAX;
							param->mission_accel_max.value=MISSION_ACCEL_MAX;
							param->alt_return.value=ALT_RETURN;
							param->voltage_gain.value=VOLTAGE_GAIN;
							param->current_gain.value=CURRENT_GAIN;
							param->uwb_yaw_delta_deg.value=UWB_YAW_DELTA_DEG;
							param->uwb_tag_id.value=UWB_TAG_ID;
							param->uwb_tag_max.value=UWB_TAG_MAX;
							param->uwb_anchor01_pos.value=Vector3f(UWB_POS1_X, UWB_POS1_Y, UWB_POS1_Z);
							param->uwb_anchor02_pos.value=Vector3f(UWB_POS2_X, UWB_POS2_Y, UWB_POS2_Z);
							param->uwb_anchor03_pos.value=Vector3f(UWB_POS3_X, UWB_POS3_Y, UWB_POS3_Z);
							param->uwb_anchor04_pos.value=Vector3f(UWB_POS4_X, UWB_POS4_Y, UWB_POS4_Z);
							param->auto_takeoff_speed.value=AUTO_TAKEOFF_SPEED;
							param->landing_lock_alt.value=LANDING_LOCK_ALT;
							param->flow_gain.value=Vector3f(FLOW_GAIN_X, FLOW_GAIN_Y, FLOW_GAIN_Z);
							param->gnss_offset.value=Vector3f(GNSS_OFFSET_X, GNSS_OFFSET_Y, GNSS_OFFSET_Z);
							param->comm1_bandrate.value=COMM_1_BANDRATE;
							param->comm2_bandrate.value=COMM_2_BANDRATE;
							param->comm3_bandrate.value=COMM_3_BANDRATE;
							param->comm4_bandrate.value=COMM_4_BANDRATE;

							dataflash->set_param_float(param->acro_y_expo.num, param->acro_y_expo.value);
							dataflash->set_param_float(param->acro_yaw_p.num, param->acro_yaw_p.value);
							dataflash->set_param_float(param->throttle_midzone.num, param->throttle_midzone.value);
							dataflash->set_param_float(param->pilot_speed_dn.num, param->pilot_speed_dn.value);
							dataflash->set_param_float(param->pilot_speed_up.num, param->pilot_speed_up.value);
							dataflash->set_param_float(param->auto_land_speed.num, param->auto_land_speed.value);
							dataflash->set_param_float(param->rangefinder_gain.num, param->rangefinder_gain.value);
							dataflash->set_param_float(param->angle_max.num, param->angle_max.value);
							dataflash->set_param_float(param->pilot_accel_z.num, param->pilot_accel_z.value);
							dataflash->set_param_float(param->pilot_takeoff_alt.num, param->pilot_takeoff_alt.value);
							dataflash->set_param_float(param->spool_up_time.num, param->spool_up_time.value);
							dataflash->set_param_float(param->throttle_filt.num, param->throttle_filt.value);
							dataflash->set_param_float(param->t_hover_update_min.num, param->t_hover_update_min.value);
							dataflash->set_param_float(param->t_hover_update_max.num, param->t_hover_update_max.value);
							dataflash->set_param_float(param->vib_land.num, param->vib_land.value);
							dataflash->set_param_float(param->lowbatt_return_volt.num, param->lowbatt_return_volt.value);
							dataflash->set_param_float(param->lowbatt_land_volt.num, param->lowbatt_land_volt.value);
							dataflash->set_param_float(param->poshold_vel_max.num, param->poshold_vel_max.value);
							dataflash->set_param_float(param->poshold_accel_max.num, param->poshold_accel_max.value);
							dataflash->set_param_float(param->mission_vel_max.num, param->mission_vel_max.value);
							dataflash->set_param_float(param->mission_accel_max.num, param->mission_accel_max.value);
							dataflash->set_param_float(param->alt_return.num, param->alt_return.value);
							dataflash->set_param_float(param->voltage_gain.num, param->voltage_gain.value);
							dataflash->set_param_float(param->current_gain.num, param->current_gain.value);
							dataflash->set_param_float(param->uwb_yaw_delta_deg.num, param->uwb_yaw_delta_deg.value);
							dataflash->set_param_uint8(param->uwb_tag_id.num, param->uwb_tag_id.value);
							dataflash->set_param_uint8(param->uwb_tag_max.num, param->uwb_tag_max.value);
							dataflash->set_param_vector3f(param->uwb_anchor01_pos.num, param->uwb_anchor01_pos.value);
							dataflash->set_param_vector3f(param->uwb_anchor02_pos.num, param->uwb_anchor02_pos.value);
							dataflash->set_param_vector3f(param->uwb_anchor03_pos.num, param->uwb_anchor03_pos.value);
							dataflash->set_param_vector3f(param->uwb_anchor04_pos.num, param->uwb_anchor04_pos.value);
							dataflash->set_param_float(param->auto_takeoff_speed.num, param->auto_takeoff_speed.value);
							dataflash->set_param_float(param->landing_lock_alt.num, param->landing_lock_alt.value);
							dataflash->set_param_vector3f(param->flow_gain.num, param->flow_gain.value);
							dataflash->set_param_vector3f(param->gnss_offset.num, param->gnss_offset.value);
							dataflash->set_param_uint32(param->comm1_bandrate.num, param->comm1_bandrate.value);
							dataflash->set_param_uint32(param->comm2_bandrate.num, param->comm2_bandrate.value);
							dataflash->set_param_uint32(param->comm3_bandrate.num, param->comm3_bandrate.value);
							dataflash->set_param_uint32(param->comm4_bandrate.num, param->comm4_bandrate.value);

							dataflash->set_param_float(param->angle_roll_p.num, param->angle_roll_p.value);
							dataflash->set_param_float(param->angle_pitch_p.num, param->angle_pitch_p.value);
							dataflash->set_param_float(param->angle_yaw_p.num, param->angle_yaw_p.value);
							dataflash->set_param_pid(param->rate_roll_pid.num, param->rate_roll_pid.value_p, param->rate_roll_pid.value_i,
										param->rate_roll_pid.value_d, param->rate_roll_pid.value_imax, param->rate_roll_pid.value_filt_hz);
							dataflash->set_param_pid(param->rate_pitch_pid.num, param->rate_pitch_pid.value_p, param->rate_pitch_pid.value_i,
										param->rate_pitch_pid.value_d, param->rate_pitch_pid.value_imax, param->rate_pitch_pid.value_filt_hz);
							dataflash->set_param_pid(param->rate_yaw_pid.num, param->rate_yaw_pid.value_p, param->rate_yaw_pid.value_i,
										param->rate_yaw_pid.value_d, param->rate_yaw_pid.value_imax, param->rate_yaw_pid.value_filt_hz);
							dataflash->set_param_float(param->pos_z_p.num, param->pos_z_p.value);
							dataflash->set_param_float(param->vel_z_p.num, param->vel_z_p.value);
							dataflash->set_param_pid(param->accel_z_pid.num, param->accel_z_pid.value_p, param->accel_z_pid.value_i,
										param->accel_z_pid.value_d, param->accel_z_pid.value_imax, param->accel_z_pid.value_filt_hz);
							dataflash->set_param_float(param->pos_xy_p.num, param->pos_xy_p.value);
							dataflash->set_param_pid_2d(param->vel_xy_pid.num, param->vel_xy_pid.value_p, param->vel_xy_pid.value_i,
										param->vel_xy_pid.value_d, param->vel_xy_pid.value_imax, param->vel_xy_pid.value_filt_hz,  param->vel_xy_pid.value_filt_d_hz);
							attitude->get_angle_roll_p()(param->angle_roll_p.value);
							attitude->get_angle_pitch_p()(param->angle_pitch_p.value);
							attitude->get_angle_yaw_p()(param->angle_yaw_p.value);
							attitude->get_rate_roll_pid()(param->rate_roll_pid.value_p, param->rate_roll_pid.value_i,
									param->rate_roll_pid.value_d, param->rate_roll_pid.value_imax, param->rate_roll_pid.value_filt_hz, _dt);
							attitude->get_rate_pitch_pid()(param->rate_pitch_pid.value_p, param->rate_pitch_pid.value_i,
									param->rate_pitch_pid.value_d, param->rate_pitch_pid.value_imax, param->rate_pitch_pid.value_filt_hz, _dt);
							attitude->get_rate_yaw_pid()(param->rate_yaw_pid.value_p, param->rate_yaw_pid.value_i,
									param->rate_yaw_pid.value_d, param->rate_yaw_pid.value_imax, param->rate_yaw_pid.value_filt_hz, _dt);
							pos_control->get_pos_z_p()(param->pos_z_p.value);
							pos_control->get_vel_z_p()(param->vel_z_p.value);
							pos_control->get_accel_z_pid()(param->accel_z_pid.value_p, param->accel_z_pid.value_i,
									param->accel_z_pid.value_d, param->accel_z_pid.value_imax, param->accel_z_pid.value_filt_hz, _dt);
							pos_control->get_pos_xy_p()(param->pos_xy_p.value);
							pos_control->get_vel_xy_pid()(param->vel_xy_pid.value_p, param->vel_xy_pid.value_p, param->vel_xy_pid.value_i, param->vel_xy_pid.value_i, param->vel_xy_pid.value_d,
									param->vel_xy_pid.value_d, param->vel_xy_pid.value_imax, param->vel_xy_pid.value_filt_hz, param->vel_xy_pid.value_filt_d_hz, _dt);
							attitude->set_lean_angle_max(param->angle_max.value);
							pos_control->set_lean_angle_max_d(param->angle_max.value);
							send_mavlink_param_list(chan);
							break;
						case 1://ANGLE_ROLL_P
							attitude->get_angle_roll_p().kP(cmd.param2);
							param->angle_roll_p.value=cmd.param2;
							dataflash->set_param_float(param->angle_roll_p.num, param->angle_roll_p.value);
							command_long.command=MAV_CMD_DO_SET_PARAMETER;
							command_long.param1=1.0f;
							command_long.param2=param->angle_roll_p.value;
							mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
							mavlink_send_buffer(chan, &msg_command_long);
							break;
						case 2://ANGLE_PITCH_P
							attitude->get_angle_pitch_p().kP(cmd.param2);
							param->angle_pitch_p.value=cmd.param2;
							dataflash->set_param_float(param->angle_pitch_p.num, param->angle_pitch_p.value);
							command_long.command=MAV_CMD_DO_SET_PARAMETER;
							command_long.param1=2.0f;
							command_long.param2=param->angle_pitch_p.value;
							mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
							mavlink_send_buffer(chan, &msg_command_long);
							break;
						case 3://ANGLE_YAW_P
							attitude->get_angle_yaw_p().kP(cmd.param2);
							param->angle_yaw_p.value=cmd.param2;
							dataflash->set_param_float(param->angle_yaw_p.num, param->angle_yaw_p.value);
							command_long.command=MAV_CMD_DO_SET_PARAMETER;
							command_long.param1=3.0f;
							command_long.param2=param->angle_yaw_p.value;
							mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
							mavlink_send_buffer(chan, &msg_command_long);
							break;
						case 4://RATE_ROLL_PID
							attitude->get_rate_roll_pid().kP(cmd.param2);
							attitude->get_rate_roll_pid().kI(cmd.param3);
							attitude->get_rate_roll_pid().kD(cmd.param4);
							attitude->get_rate_roll_pid().imax(cmd.param5);
							attitude->get_rate_roll_pid().filt_hz(cmd.param6);
							param->rate_roll_pid.value_p=cmd.param2;
							param->rate_roll_pid.value_i=cmd.param3;
							param->rate_roll_pid.value_d=cmd.param4;
							param->rate_roll_pid.value_imax=cmd.param5;
							param->rate_roll_pid.value_filt_hz=cmd.param6;
							dataflash->set_param_pid(param->rate_roll_pid.num, param->rate_roll_pid.value_p, param->rate_roll_pid.value_i,
									param->rate_roll_pid.value_d, param->rate_roll_pid.value_imax, param->rate_roll_pid.value_filt_hz);
							command_long.command=MAV_CMD_DO_SET_PARAMETER;
							command_long.param1=4.0f;
							command_long.param2=param->rate_roll_pid.value_p;
							command_long.param3=param->rate_roll_pid.value_i;
							command_long.param4=param->rate_roll_pid.value_d;
							command_long.param5=param->rate_roll_pid.value_imax;
							command_long.param6=param->rate_roll_pid.value_filt_hz;
							mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
							mavlink_send_buffer(chan, &msg_command_long);
							break;
						case 5://RATE_PITCH_PID
							attitude->get_rate_pitch_pid().kP(cmd.param2);
							attitude->get_rate_pitch_pid().kI(cmd.param3);
							attitude->get_rate_pitch_pid().kD(cmd.param4);
							attitude->get_rate_pitch_pid().imax(cmd.param5);
							attitude->get_rate_pitch_pid().filt_hz(cmd.param6);
							param->rate_pitch_pid.value_p=cmd.param2;
							param->rate_pitch_pid.value_i=cmd.param3;
							param->rate_pitch_pid.value_d=cmd.param4;
							param->rate_pitch_pid.value_imax=cmd.param5;
							param->rate_pitch_pid.value_filt_hz=cmd.param6;
							dataflash->set_param_pid(param->rate_pitch_pid.num, param->rate_pitch_pid.value_p, param->rate_pitch_pid.value_i,
									param->rate_pitch_pid.value_d, param->rate_pitch_pid.value_imax, param->rate_pitch_pid.value_filt_hz);
							command_long.command=MAV_CMD_DO_SET_PARAMETER;
							command_long.param1=5.0f;
							command_long.param2=param->rate_pitch_pid.value_p;
							command_long.param3=param->rate_pitch_pid.value_i;
							command_long.param4=param->rate_pitch_pid.value_d;
							command_long.param5=param->rate_pitch_pid.value_imax;
							command_long.param6=param->rate_pitch_pid.value_filt_hz;
							mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
							mavlink_send_buffer(chan, &msg_command_long);
							break;
						case 6://RATE_YAW_PID
							attitude->get_rate_yaw_pid().kP(cmd.param2);
							attitude->get_rate_yaw_pid().kI(cmd.param3);
							attitude->get_rate_yaw_pid().kD(cmd.param4);
							attitude->get_rate_yaw_pid().imax(cmd.param5);
							attitude->get_rate_yaw_pid().filt_hz(cmd.param6);
							param->rate_yaw_pid.value_p=cmd.param2;
							param->rate_yaw_pid.value_i=cmd.param3;
							param->rate_yaw_pid.value_d=cmd.param4;
							param->rate_yaw_pid.value_imax=cmd.param5;
							param->rate_yaw_pid.value_filt_hz=cmd.param6;
							dataflash->set_param_pid(param->rate_yaw_pid.num, param->rate_yaw_pid.value_p, param->rate_yaw_pid.value_i,
									param->rate_yaw_pid.value_d, param->rate_yaw_pid.value_imax, param->rate_yaw_pid.value_filt_hz);
							command_long.command=MAV_CMD_DO_SET_PARAMETER;
							command_long.param1=6.0f;
							command_long.param2=param->rate_yaw_pid.value_p;
							command_long.param3=param->rate_yaw_pid.value_i;
							command_long.param4=param->rate_yaw_pid.value_d;
							command_long.param5=param->rate_yaw_pid.value_imax;
							command_long.param6=param->rate_yaw_pid.value_filt_hz;
							mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
							mavlink_send_buffer(chan, &msg_command_long);
							break;
						case 7://POS_Z_P
							pos_control->get_pos_z_p().kP(cmd.param2);
							param->pos_z_p.value=cmd.param2;
							dataflash->set_param_float(param->pos_z_p.num, param->pos_z_p.value);
							command_long.command=MAV_CMD_DO_SET_PARAMETER;
							command_long.param1=7.0f;
							command_long.param2=param->pos_z_p.value;
							mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
							mavlink_send_buffer(chan, &msg_command_long);
							break;
						case 8://VEL_Z_P
							pos_control->get_vel_z_p().kP(cmd.param2);
							param->vel_z_p.value=cmd.param2;
							dataflash->set_param_float(param->vel_z_p.num, param->vel_z_p.value);
							command_long.command=MAV_CMD_DO_SET_PARAMETER;
							command_long.param1=8.0f;
							command_long.param2=param->vel_z_p.value;
							mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
							mavlink_send_buffer(chan, &msg_command_long);
							break;
						case 9://ACCEL_Z_PID
							pos_control->get_accel_z_pid().kP(cmd.param2);
							pos_control->get_accel_z_pid().kI(cmd.param3);
							pos_control->get_accel_z_pid().kD(cmd.param4);
							pos_control->get_accel_z_pid().imax(cmd.param5);
							pos_control->get_accel_z_pid().filt_hz(cmd.param6);
							param->accel_z_pid.value_p=cmd.param2;
							param->accel_z_pid.value_i=cmd.param3;
							param->accel_z_pid.value_d=cmd.param4;
							param->accel_z_pid.value_imax=cmd.param5;
							param->accel_z_pid.value_filt_hz=cmd.param6;
							dataflash->set_param_pid(param->accel_z_pid.num, param->accel_z_pid.value_p, param->accel_z_pid.value_i,
									param->accel_z_pid.value_d, param->accel_z_pid.value_imax, param->accel_z_pid.value_filt_hz);
							command_long.command=MAV_CMD_DO_SET_PARAMETER;
							command_long.param1=9.0f;
							command_long.param2=param->accel_z_pid.value_p;
							command_long.param3=param->accel_z_pid.value_i;
							command_long.param4=param->accel_z_pid.value_d;
							command_long.param5=param->accel_z_pid.value_imax;
							command_long.param6=param->accel_z_pid.value_filt_hz;
							mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
							mavlink_send_buffer(chan, &msg_command_long);
							break;
						case 10://POS_XY_P
							pos_control->get_pos_xy_p().kP(cmd.param2);
							param->pos_xy_p.value=cmd.param2;
							dataflash->set_param_float(param->pos_xy_p.num, param->pos_xy_p.value);
							command_long.command=MAV_CMD_DO_SET_PARAMETER;
							command_long.param1=10.0f;
							command_long.param2=param->pos_xy_p.value;
							mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
							mavlink_send_buffer(chan, &msg_command_long);
							break;
						case 11://VEL_XY_PID
							pos_control->get_vel_xy_pid().kP(Vector2f(cmd.param2,cmd.param2));
							pos_control->get_vel_xy_pid().kI(Vector2f(cmd.param3,cmd.param3));
							pos_control->get_vel_xy_pid().kD(Vector2f(cmd.param4,cmd.param4));
							pos_control->get_vel_xy_pid().imax(cmd.param5);
							pos_control->get_vel_xy_pid().filt_hz(cmd.param6);
							pos_control->get_vel_xy_pid().filt_d_hz(cmd.param7);
							param->vel_xy_pid.value_p=cmd.param2;
							param->vel_xy_pid.value_i=cmd.param3;
							param->vel_xy_pid.value_d=cmd.param4;
							param->vel_xy_pid.value_imax=cmd.param5;
							param->vel_xy_pid.value_filt_hz=cmd.param6;
							param->vel_xy_pid.value_filt_d_hz=cmd.param7;
							dataflash->set_param_pid_2d(param->vel_xy_pid.num, param->vel_xy_pid.value_p, param->vel_xy_pid.value_i,
									param->vel_xy_pid.value_d, param->vel_xy_pid.value_imax, param->vel_xy_pid.value_filt_hz, param->vel_xy_pid.value_filt_d_hz);
							command_long.command=MAV_CMD_DO_SET_PARAMETER;
							command_long.param1=11.0f;
							command_long.param2=param->vel_xy_pid.value_p;
							command_long.param3=param->vel_xy_pid.value_i;
							command_long.param4=param->vel_xy_pid.value_d;
							command_long.param5=param->vel_xy_pid.value_imax;
							command_long.param6=param->vel_xy_pid.value_filt_hz;
							command_long.param7=param->vel_xy_pid.value_filt_d_hz;
							mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
							mavlink_send_buffer(chan, &msg_command_long);
							break;
						case 12:
							param->acro_y_expo.value=cmd.param2;
							dataflash->set_param_float(param->acro_y_expo.num, param->acro_y_expo.value);
							command_long.command=MAV_CMD_DO_SET_PARAMETER;
							command_long.param1=12.0f;
							command_long.param2=param->acro_y_expo.value;
							mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
							mavlink_send_buffer(chan, &msg_command_long);
							break;
						case 13:
							param->acro_yaw_p.value=cmd.param2;
							dataflash->set_param_float(param->acro_yaw_p.num, param->acro_yaw_p.value);
							command_long.command=MAV_CMD_DO_SET_PARAMETER;
							command_long.param1=13.0f;
							command_long.param2=param->acro_yaw_p.value;
							mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
							mavlink_send_buffer(chan, &msg_command_long);
							break;
						case 14:
							param->throttle_midzone.value=cmd.param2;
							dataflash->set_param_float(param->throttle_midzone.num, param->throttle_midzone.value);
							command_long.command=MAV_CMD_DO_SET_PARAMETER;
							command_long.param1=14.0f;
							command_long.param2=param->throttle_midzone.value;
							mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
							mavlink_send_buffer(chan, &msg_command_long);
							break;
						case 15:
							param->pilot_speed_dn.value=cmd.param2;
							dataflash->set_param_float(param->pilot_speed_dn.num, param->pilot_speed_dn.value);
							command_long.command=MAV_CMD_DO_SET_PARAMETER;
							command_long.param1=15.0f;
							command_long.param2=param->pilot_speed_dn.value;
							mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
							mavlink_send_buffer(chan, &msg_command_long);
							break;
						case 16:
							param->pilot_speed_up.value=cmd.param2;
							dataflash->set_param_float(param->pilot_speed_up.num, param->pilot_speed_up.value);
							command_long.command=MAV_CMD_DO_SET_PARAMETER;
							command_long.param1=16.0f;
							command_long.param2=param->pilot_speed_up.value;
							mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
							mavlink_send_buffer(chan, &msg_command_long);
							break;
						case 17:
							param->rangefinder_gain.value=cmd.param2;
							dataflash->set_param_float(param->rangefinder_gain.num, param->rangefinder_gain.value);
							command_long.command=MAV_CMD_DO_SET_PARAMETER;
							command_long.param1=17.0f;
							command_long.param2=param->rangefinder_gain.value;
							mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
							mavlink_send_buffer(chan, &msg_command_long);
							break;
						case 18:
							param->angle_max.value=cmd.param2;
							attitude->set_lean_angle_max(param->angle_max.value);
							pos_control->set_lean_angle_max_d(param->angle_max.value);
							dataflash->set_param_float(param->angle_max.num, param->angle_max.value);
							command_long.command=MAV_CMD_DO_SET_PARAMETER;
							command_long.param1=18.0f;
							command_long.param2=param->angle_max.value;
							mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
							mavlink_send_buffer(chan, &msg_command_long);
							break;
						case 19:
							param->pilot_accel_z.value=cmd.param2;
							dataflash->set_param_float(param->pilot_accel_z.num, param->pilot_accel_z.value);
							command_long.command=MAV_CMD_DO_SET_PARAMETER;
							command_long.param1=19.0f;
							command_long.param2=param->pilot_accel_z.value;
							mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
							mavlink_send_buffer(chan, &msg_command_long);
							break;
						case 20:
							param->pilot_takeoff_alt.value=cmd.param2;
							dataflash->set_param_float(param->pilot_takeoff_alt.num, param->pilot_takeoff_alt.value);
							command_long.command=MAV_CMD_DO_SET_PARAMETER;
							command_long.param1=20.0f;
							command_long.param2=param->pilot_takeoff_alt.value;
							mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
							mavlink_send_buffer(chan, &msg_command_long);
							break;
						case 21:
							param->spool_up_time.value=cmd.param2;
							dataflash->set_param_float(param->spool_up_time.num, param->spool_up_time.value);
							command_long.command=MAV_CMD_DO_SET_PARAMETER;
							command_long.param1=21.0f;
							command_long.param2=param->spool_up_time.value;
							mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
							mavlink_send_buffer(chan, &msg_command_long);
							break;
						case 22:
							param->throttle_filt.value=cmd.param2;
							dataflash->set_param_float(param->throttle_filt.num, param->throttle_filt.value);
							command_long.command=MAV_CMD_DO_SET_PARAMETER;
							command_long.param1=22.0f;
							command_long.param2=param->throttle_filt.value;
							mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
							mavlink_send_buffer(chan, &msg_command_long);
							break;
						case 23:
							param->t_hover_update_min.value=cmd.param2;
							dataflash->set_param_float(param->t_hover_update_min.num, param->t_hover_update_min.value);
							command_long.command=MAV_CMD_DO_SET_PARAMETER;
							command_long.param1=23.0f;
							command_long.param2=param->t_hover_update_min.value;
							mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
							mavlink_send_buffer(chan, &msg_command_long);
							break;
						case 24:
							param->t_hover_update_max.value=cmd.param2;
							dataflash->set_param_float(param->t_hover_update_max.num, param->t_hover_update_max.value);
							command_long.command=MAV_CMD_DO_SET_PARAMETER;
							command_long.param1=24.0f;
							command_long.param2=param->t_hover_update_max.value;
							mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
							mavlink_send_buffer(chan, &msg_command_long);
							break;
						case 25:
							param->vib_land.value=cmd.param2;
							dataflash->set_param_float(param->vib_land.num, param->vib_land.value);
							command_long.command=MAV_CMD_DO_SET_PARAMETER;
							command_long.param1=25.0f;
							command_long.param2=param->vib_land.value;
							mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
							mavlink_send_buffer(chan, &msg_command_long);
							break;
						case 26:
							param->auto_land_speed.value=cmd.param2;
							dataflash->set_param_float(param->auto_land_speed.num, param->auto_land_speed.value);
							command_long.command=MAV_CMD_DO_SET_PARAMETER;
							command_long.param1=26.0f;
							command_long.param2=param->auto_land_speed.value;
							mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
							mavlink_send_buffer(chan, &msg_command_long);
							break;
						case 27:
							param->lowbatt_return_volt.value=cmd.param2;
							dataflash->set_param_float(param->lowbatt_return_volt.num, param->lowbatt_return_volt.value);
							command_long.command=MAV_CMD_DO_SET_PARAMETER;
							command_long.param1=27.0f;
							command_long.param2=param->lowbatt_return_volt.value;
							mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
							mavlink_send_buffer(chan, &msg_command_long);
							break;
						case 28:
							param->lowbatt_land_volt.value=cmd.param2;
							dataflash->set_param_float(param->lowbatt_land_volt.num, param->lowbatt_land_volt.value);
							command_long.command=MAV_CMD_DO_SET_PARAMETER;
							command_long.param1=28.0f;
							command_long.param2=param->lowbatt_land_volt.value;
							mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
							mavlink_send_buffer(chan, &msg_command_long);
							break;
						case 29:
							param->poshold_vel_max.value=cmd.param2;
							dataflash->set_param_float(param->poshold_vel_max.num, param->poshold_vel_max.value);
							command_long.command=MAV_CMD_DO_SET_PARAMETER;
							command_long.param1=29.0f;
							command_long.param2=param->poshold_vel_max.value;
							mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
							mavlink_send_buffer(chan, &msg_command_long);
							break;
						case 30:
							param->poshold_accel_max.value=cmd.param2;
							dataflash->set_param_float(param->poshold_accel_max.num, param->poshold_accel_max.value);
							command_long.command=MAV_CMD_DO_SET_PARAMETER;
							command_long.param1=30.0f;
							command_long.param2=param->poshold_accel_max.value;
							mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
							mavlink_send_buffer(chan, &msg_command_long);
							break;
						case 31:
							param->mission_vel_max.value=cmd.param2;
							dataflash->set_param_float(param->mission_vel_max.num, param->mission_vel_max.value);
							command_long.command=MAV_CMD_DO_SET_PARAMETER;
							command_long.param1=31.0f;
							command_long.param2=param->mission_vel_max.value;
							mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
							mavlink_send_buffer(chan, &msg_command_long);
							break;
						case 32:
							param->mission_accel_max.value=cmd.param2;
							dataflash->set_param_float(param->mission_accel_max.num, param->mission_accel_max.value);
							command_long.command=MAV_CMD_DO_SET_PARAMETER;
							command_long.param1=32.0f;
							command_long.param2=param->mission_accel_max.value;
							mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
							mavlink_send_buffer(chan, &msg_command_long);
							break;
						case 33:
							param->alt_return.value=cmd.param2;
							dataflash->set_param_float(param->alt_return.num, param->alt_return.value);
							command_long.command=MAV_CMD_DO_SET_PARAMETER;
							command_long.param1=33.0f;
							command_long.param2=param->alt_return.value;
							mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
							mavlink_send_buffer(chan, &msg_command_long);
							break;
						case 34:
							param->voltage_gain.value=cmd.param2;
							dataflash->set_param_float(param->voltage_gain.num, param->voltage_gain.value);
							command_long.command=MAV_CMD_DO_SET_PARAMETER;
							command_long.param1=34.0f;
							command_long.param2=param->voltage_gain.value;
							mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
							mavlink_send_buffer(chan, &msg_command_long);
							break;
						case 35:
							param->current_gain.value=cmd.param2;
							dataflash->set_param_float(param->current_gain.num, param->current_gain.value);
							command_long.command=MAV_CMD_DO_SET_PARAMETER;
							command_long.param1=35.0f;
							command_long.param2=param->current_gain.value;
							mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
							mavlink_send_buffer(chan, &msg_command_long);
							break;
						case 36:
							param->uwb_yaw_delta_deg.value=cmd.param2;
							dataflash->set_param_float(param->uwb_yaw_delta_deg.num, param->uwb_yaw_delta_deg.value);
							command_long.command=MAV_CMD_DO_SET_PARAMETER;
							command_long.param1=36.0f;
							command_long.param2=param->uwb_yaw_delta_deg.value;
							uwb_yaw_delta=-param->uwb_yaw_delta_deg.value*DEG_TO_RAD;
							mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
							mavlink_send_buffer(chan, &msg_command_long);
							break;
						case 37:
							param->uwb_tag_id.value=(uint8_t)cmd.param2;
							dataflash->set_param_uint8(param->uwb_tag_id.num, param->uwb_tag_id.value);
							command_long.command=MAV_CMD_DO_SET_PARAMETER;
							command_long.param1=37.0f;
							command_long.param2=(float)param->uwb_tag_id.value;
							mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
							mavlink_send_buffer(chan, &msg_command_long);
							break;
						case 38:
							param->uwb_tag_max.value=(uint8_t)cmd.param2;
							dataflash->set_param_uint8(param->uwb_tag_max.num, param->uwb_tag_max.value);
							command_long.command=MAV_CMD_DO_SET_PARAMETER;
							command_long.param1=38.0f;
							command_long.param2=(float)param->uwb_tag_max.value;
							mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
							mavlink_send_buffer(chan, &msg_command_long);
							break;
						case 39:
							param->uwb_anchor01_pos.value.x=cmd.param2;
							param->uwb_anchor01_pos.value.y=cmd.param3;
							param->uwb_anchor01_pos.value.z=cmd.param4;
							dataflash->set_param_vector3f(param->uwb_anchor01_pos.num, param->uwb_anchor01_pos.value);
							uwb->set_anchor_positon(1, param->uwb_anchor01_pos.value.x, param->uwb_anchor01_pos.value.y, param->uwb_anchor01_pos.value.z);
							command_long.command=MAV_CMD_DO_SET_PARAMETER;
							command_long.param1=39.0f;
							command_long.param2=param->uwb_anchor01_pos.value.x;
							command_long.param3=param->uwb_anchor01_pos.value.y;
							command_long.param4=param->uwb_anchor01_pos.value.z;
							mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
							mavlink_send_buffer(chan, &msg_command_long);
							break;
						case 40:
							param->uwb_anchor02_pos.value.x=cmd.param2;
							param->uwb_anchor02_pos.value.y=cmd.param3;
							param->uwb_anchor02_pos.value.z=cmd.param4;
							dataflash->set_param_vector3f(param->uwb_anchor02_pos.num, param->uwb_anchor02_pos.value);
							uwb->set_anchor_positon(2, param->uwb_anchor02_pos.value.x, param->uwb_anchor02_pos.value.y, param->uwb_anchor02_pos.value.z);
							command_long.command=MAV_CMD_DO_SET_PARAMETER;
							command_long.param1=40.0f;
							command_long.param2=param->uwb_anchor02_pos.value.x;
							command_long.param3=param->uwb_anchor02_pos.value.y;
							command_long.param4=param->uwb_anchor02_pos.value.z;
							mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
							mavlink_send_buffer(chan, &msg_command_long);
							break;
						case 41:
							param->uwb_anchor03_pos.value.x=cmd.param2;
							param->uwb_anchor03_pos.value.y=cmd.param3;
							param->uwb_anchor03_pos.value.z=cmd.param4;
							dataflash->set_param_vector3f(param->uwb_anchor03_pos.num, param->uwb_anchor03_pos.value);
							uwb->set_anchor_positon(3, param->uwb_anchor03_pos.value.x, param->uwb_anchor03_pos.value.y, param->uwb_anchor03_pos.value.z);
							command_long.command=MAV_CMD_DO_SET_PARAMETER;
							command_long.param1=41.0f;
							command_long.param2=param->uwb_anchor03_pos.value.x;
							command_long.param3=param->uwb_anchor03_pos.value.y;
							command_long.param4=param->uwb_anchor03_pos.value.z;
							mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
							mavlink_send_buffer(chan, &msg_command_long);
							break;
						case 42:
							param->uwb_anchor04_pos.value.x=cmd.param2;
							param->uwb_anchor04_pos.value.y=cmd.param3;
							param->uwb_anchor04_pos.value.z=cmd.param4;
							dataflash->set_param_vector3f(param->uwb_anchor04_pos.num, param->uwb_anchor04_pos.value);
							uwb->set_anchor_positon(4, param->uwb_anchor04_pos.value.x, param->uwb_anchor04_pos.value.y, param->uwb_anchor04_pos.value.z);
							command_long.command=MAV_CMD_DO_SET_PARAMETER;
							command_long.param1=42.0f;
							command_long.param2=param->uwb_anchor04_pos.value.x;
							command_long.param3=param->uwb_anchor04_pos.value.y;
							command_long.param4=param->uwb_anchor04_pos.value.z;
							mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
							mavlink_send_buffer(chan, &msg_command_long);
							break;
						case 43:
							param->auto_takeoff_speed.value=cmd.param2;
							dataflash->set_param_float(param->auto_takeoff_speed.num, param->auto_takeoff_speed.value);
							command_long.command=MAV_CMD_DO_SET_PARAMETER;
							command_long.param1=43.0f;
							command_long.param2=param->auto_takeoff_speed.value;
							mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
							mavlink_send_buffer(chan, &msg_command_long);
							break;
						case 44:
							param->landing_lock_alt.value=cmd.param2;
							dataflash->set_param_float(param->landing_lock_alt.num, param->landing_lock_alt.value);
							command_long.command=MAV_CMD_DO_SET_PARAMETER;
							command_long.param1=44.0f;
							command_long.param2=param->landing_lock_alt.value;
							mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
							mavlink_send_buffer(chan, &msg_command_long);
							break;
						case 45:
							param->flow_gain.value.x=cmd.param2;
							param->flow_gain.value.y=cmd.param3;
							param->flow_gain.value.z=cmd.param4;
							dataflash->set_param_vector3f(param->flow_gain.num, param->flow_gain.value);
							command_long.command=MAV_CMD_DO_SET_PARAMETER;
							command_long.param1=45.0f;
							command_long.param2=param->flow_gain.value.x;
							command_long.param3=param->flow_gain.value.y;
							command_long.param4=param->flow_gain.value.z;
							mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
							mavlink_send_buffer(chan, &msg_command_long);
							break;
						case 46:
							param->gnss_offset.value.x=cmd.param2;
							param->gnss_offset.value.y=cmd.param3;
							param->gnss_offset.value.z=cmd.param4;
							dataflash->set_param_vector3f(param->gnss_offset.num, param->gnss_offset.value);
							command_long.command=MAV_CMD_DO_SET_PARAMETER;
							command_long.param1=46.0f;
							command_long.param2=param->gnss_offset.value.x;
							command_long.param3=param->gnss_offset.value.y;
							command_long.param4=param->gnss_offset.value.z;
							mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
							mavlink_send_buffer(chan, &msg_command_long);
							break;
						case 47:
							if(gcs_channel!=MAVLINK_COMM_1){
								param->comm1_bandrate.value=(uint32_t)cmd.param2;
								set_s1_baudrate(param->comm1_bandrate.value);
								dataflash->set_param_uint32(param->comm1_bandrate.num, param->comm1_bandrate.value);
							}
							command_long.command=MAV_CMD_DO_SET_PARAMETER;
							command_long.param1=47.0f;
							command_long.param2=(float)param->comm1_bandrate.value;//波特率
							mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
							mavlink_send_buffer(chan, &msg_command_long);
							break;
						case 48:
							if(gcs_channel!=MAVLINK_COMM_2){
								param->comm2_bandrate.value=(uint32_t)cmd.param2;
								set_s2_baudrate(param->comm2_bandrate.value);
								dataflash->set_param_uint32(param->comm2_bandrate.num, param->comm2_bandrate.value);
							}
							command_long.command=MAV_CMD_DO_SET_PARAMETER;
							command_long.param1=48.0f;
							command_long.param2=(float)param->comm2_bandrate.value;//波特率
							mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
							mavlink_send_buffer(chan, &msg_command_long);
							break;
						case 49:
							if(gcs_channel!=MAVLINK_COMM_3){
								param->comm3_bandrate.value=(uint32_t)cmd.param2;
								set_s3_baudrate(param->comm3_bandrate.value);
								dataflash->set_param_uint32(param->comm3_bandrate.num, param->comm3_bandrate.value);
							}
							command_long.command=MAV_CMD_DO_SET_PARAMETER;
							command_long.param1=49.0f;
							command_long.param2=(float)param->comm3_bandrate.value;//波特率
							mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
							mavlink_send_buffer(chan, &msg_command_long);
							break;
						case 50:
							if(gcs_channel!=MAVLINK_COMM_4){
								param->comm4_bandrate.value=(uint32_t)cmd.param2;
								set_s4_baudrate(param->comm4_bandrate.value);
								dataflash->set_param_uint32(param->comm4_bandrate.num, param->comm4_bandrate.value);
							}
							command_long.command=MAV_CMD_DO_SET_PARAMETER;
							command_long.param1=50.0f;
							command_long.param2=(float)param->comm4_bandrate.value;//波特率
							mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
							mavlink_send_buffer(chan, &msg_command_long);
							break;
						/* *************************************************
						 * ****************Dev code begin*******************/
						// Warning! Developer can add your new code here!
						/* Demo
						 * 接收app设置的参数值
						 case 1001: 		//cmd.param1为自定义参数的mavlink id, 从1001开始
							param->demo_param_1.value.x=cmd.param2;		//cmd.param2~cmd.param7为参数实际内容。
							param->demo_param_1.value.y=cmd.param3;
							param->demo_param_1.value.z=cmd.param4;
							dataflash->set_param_vector3f(param->demo_param_1.num, param->demo_param_1.value);
							//接收完参数还要把参数传回给app用于校验
							command_long.command=MAV_CMD_DO_SET_PARAMETER;
							command_long.param1=1001.0f;
							command_long.param2=param->demo_param_1.value.x;
							command_long.param3=param->demo_param_1.value.y;
							command_long.param4=param->demo_param_1.value.z;
							mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
							mavlink_send_buffer(chan, &msg_command_long);
							break;
						 * */

						/* ****************Dev code end*********************
						 * *************************************************/
						default:
							break;
						}
						break;
					default:
						break;
				}
				break;
			case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
				send_mavlink_param_list(chan);
				break;
			case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:
				mavlink_msg_rc_channels_override_decode(msg_received, &rc_channels);
				if(rc_channels.target_component==0){			/***APP遥控功能已启用***/
					mav_channels_in[0]=rc_channels.chan1_raw;
					mav_channels_in[1]=rc_channels.chan2_raw;
					mav_channels_in[2]=rc_channels.chan3_raw;
					mav_channels_in[3]=rc_channels.chan4_raw;
					mav_channels_in[4]=rc_channels.chan5_raw;
					mav_channels_in[5]=rc_channels.chan6_raw;
					mav_channels_in[6]=rc_channels.chan7_raw;
					mav_channels_in[7]=rc_channels.chan8_raw;
					if(chan==gcs_channel){
						set_rc_channels_override(true);				//使能rc_channels_override把控制权给APP
					}
					rc_channels_sendback=false;					//关闭遥控通道回传
				}else if(rc_channels.target_component==1){		/***遥控校准已确认***/
					set_rc_channels_override(false);			//清除rc_channels_override把控制权给遥控器
					rc_channels_sendback=true;					//启动遥控通道回传
					param->channel_range.channel[0]=rc_channels.chan1_raw;	//ch1_min
					param->channel_range.channel[1]=rc_channels.chan2_raw;	//ch2_min
					param->channel_range.channel[2]=rc_channels.chan3_raw;	//ch3_min
					param->channel_range.channel[3]=rc_channels.chan4_raw;	//ch4_min
					param->channel_range.channel[4]=rc_channels.chan5_raw;	//ch1_max
					param->channel_range.channel[5]=rc_channels.chan6_raw;	//ch2_max
					param->channel_range.channel[6]=rc_channels.chan7_raw;	//ch3_max
					param->channel_range.channel[7]=rc_channels.chan8_raw;	//ch4_max
					dataflash->set_param_uint16_channel8(param->channel_range.num, param->channel_range.channel);
					rc_range_init();
					send_mavlink_commond_ack(chan, (MAV_CMD)1, MAV_CMD_ACK_OK);
				}else if(rc_channels.target_component==2){		/***遥控校准已启动***/
					mav_channels_in[0]=1500;
					mav_channels_in[1]=1500;
					mav_channels_in[2]=1500;
					mav_channels_in[3]=1500;
					override_rc_channels(mav_channels_in);		//重置前四通道
					set_rc_channels_override(false);			//清除rc_channels_override把控制权给遥控器
					rc_channels_sendback=true;					//启动遥控通道回传
					send_mavlink_commond_ack(chan, (MAV_CMD)2, MAV_CMD_ACK_OK);
				}else{											/***APP遥控功能已禁用***/
					set_rc_channels_override(false);			//清除rc_channels_override把控制权给遥控器
					rc_channels_sendback=false;					//关闭遥控通道回传
					send_mavlink_commond_ack(chan, (MAV_CMD)3, MAV_CMD_ACK_OK);
				}
				break;
			case MAVLINK_MSG_ID_ATTITUDE:   // MAV ID: 30
				mavlink_msg_attitude_decode(msg_received, &attitude_mav);
				yaw_map=wrap_PI(attitude_mav.yaw);  //rad 外部传入的偏航角必须z轴向下的弧度角,即NED或FRD坐标系
				if(time_last_attitude==0){
					ahrs->reset();
				}
				time_last_attitude=HAL_GetTick();
				if(!USE_MAG&&enable_odom&&odom_safe){
					get_mav_yaw=true;
				}
				break;
			case MAVLINK_MSG_ID_LOCAL_POSITION_NED_COV:       // MAV ID: 64
				mavlink_msg_local_position_ned_cov_decode(msg_received, &local_position_ned_cov);
				odom_offset=dcm_matrix*lidar_offset;
				odom_3d.x=local_position_ned_cov.x * 100.0f-odom_offset.x;  //cm 外部定位必须是NED或者FRD坐标系,如果是FRD坐标还需要禁用磁罗盘。
				odom_3d.y=local_position_ned_cov.y * 100.0f-odom_offset.y;  //cm
				if(USE_ODOM_Z){
					odom_3d.z=-local_position_ned_cov.z * 100.0f; //cm
					odom_dz=odom_3d.z-odom_z_last;
					if(USE_VINS){
						odomz_tc=constrain_float(vins_tc, 0.0, 1.0);
						odomz_dt=constrain_float(vins_dt,0.1,0.2);
					}else{
						odomz_tc=constrain_float(lidar_tc, 0.0, 1.0);
						odomz_dt=constrain_float(lidar_dt,0.1,0.2);
					}
					if(use_rangefinder&&odom_safe&&!is_equal(odom_3d.z, 0.0f)&&!is_equal(odom_3d.z, odom_z_last)&&fabs(odom_dz)<100.0f){
						rangefinder_state.enabled=true;
						rangefinder_state.alt_healthy=true;
						rangefinder_state.alt_cm=rangefinder_state.alt_cm_filt.apply(odom_3d.z+odom_dz/odomz_dt*odomz_tc, odomz_dt);
//						usb_printf("odom:%f|%f|%f|%f\n",odom_3d.z,odom_dz,odomz_dt, rangefinder_state.alt_cm);
						rangefinder_state.last_update_ms=HAL_GetTick();
					}else{
						rangefinder_state.alt_healthy=false;
						rangefinder_state.enabled=false;
					}
					odom_z_last=odom_3d.z;
				}
				usb_printf("odom:%f|%f|%f|%f\n",odom_3d.x,odom_3d.y,odom_3d.z,yaw_map*RAD_TO_DEG);
				odom_dx=odom_3d.x-odom_2d_x_last;
				odom_dy=odom_3d.y-odom_2d_y_last;
				odom_2d=safe_sqrt(odom_dx*odom_dx+odom_dy*odom_dy);
				if(USE_MOTION&&odom_2d==0){
					odom_2d=0.0001;
				}
				odom_2d_x_last=odom_3d.x;
				odom_2d_y_last=odom_3d.y;
				if(!USE_MAG&&enable_odom&&odom_safe){
					get_odom_xy=true;
					update_odom_xy=true;
				}
				get_odom_time=HAL_GetTick();
				break;
			case MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED:     // MAV ID: 84
				mavlink_msg_set_position_target_local_ned_decode(msg_received, &set_position_target_local_ned);
				switch(set_position_target_local_ned.coordinate_frame){
				case MAV_FRAME_BODY_NED:
					mav_ax_roll_target=set_position_target_local_ned.afx*RAD_TO_DEG;
					mav_ay_pitch_target=set_position_target_local_ned.afy*RAD_TO_DEG;
					break;
				case MAV_FRAME_MISSION:
					if(offboard_connected){
						set_goal_point.time_boot_ms=HAL_GetTick();
						set_goal_point.x=set_position_target_local_ned.x;
						set_goal_point.y=set_position_target_local_ned.y;
						set_goal_point.z=set_position_target_local_ned.z;
						set_goal_point.coordinate_frame=MAV_FRAME_MISSION;
//						usb_printf("pos:%f,%f,%f\n",set_goal_point.x,set_goal_point.y,set_goal_point.z);
						mavlink_msg_set_position_target_local_ned_encode(mavlink_system.sysid, mavlink_system.compid, &msg_set_goal_point, &set_goal_point);
						mavlink_send_buffer((mavlink_channel_t)offboard_channel, &msg_set_goal_point);
					}
					break;
				default:
					mav_x_target=set_position_target_local_ned.x * 100.0f;//接收的外部目标必须是NED或者FRD坐标系,单位是m,先转为cm
					mav_y_target=set_position_target_local_ned.y * 100.0f;
					mav_z_target=set_position_target_local_ned.z * 100.0f;
					mav_vx_target=set_position_target_local_ned.vx * 100.0f;
					mav_vy_target=set_position_target_local_ned.vy * 100.0f;
					mav_vz_target=set_position_target_local_ned.vz * 100.0f;
					mav_ax_target=set_position_target_local_ned.afx * 100.0f;
					mav_ay_target=set_position_target_local_ned.afy * 100.0f;
					mav_az_target=set_position_target_local_ned.afz * 100.0f;
					mav_yaw_target=set_position_target_local_ned.yaw*RAD_TO_DEG;
					mav_yaw_rate_target=set_position_target_local_ned.yaw_rate*RAD_TO_DEG;
					get_mav_target=true;
					break;
				}
				break;
			case  MAVLINK_MSG_ID_GLOBAL_VISION_POSITION_ESTIMATE:
				mavlink_msg_global_vision_position_estimate_decode(msg_received, &pose);
//				usb_printf("pose:%f|%f|%f|%d\n",pose.x,pose.y,pose.z,HAL_GetTick());
				break;
			default:
				break;
		}
	}
}

void send_mavlink_goal_point(float x, float y, float z){
	if(offboard_connected){
		set_goal_point.time_boot_ms=HAL_GetTick();
		set_goal_point.x=x;
		set_goal_point.y=y;
		set_goal_point.z=z;
		set_goal_point.coordinate_frame=MAV_FRAME_MISSION;
//		usb_printf("pos:%f,%f,%f\n",set_goal_point.x,set_goal_point.y,set_goal_point.z);
		mavlink_msg_set_position_target_local_ned_encode(mavlink_system.sysid, mavlink_system.compid, &msg_set_goal_point, &set_goal_point);
		mavlink_send_buffer((mavlink_channel_t)offboard_channel, &msg_set_goal_point);
	}
}

//系统启动后必须确保心跳函数1s运行一次
void send_mavlink_heartbeat_data(void){
	mavlink_system.sysid=1;
	mavlink_system.compid=MAV_COMP_ID_AUTOPILOT1;
	heartbeat_send.type=param->robot_type.value;//机器人类型
	heartbeat_send.system_status=param->motor_type.value;//电机类型
	heartbeat_send.autopilot=robot_main_mode;//主模式
	heartbeat_send.custom_mode=robot_sub_mode;//子模式
	heartbeat_send.base_mode=0;
	if(motors->get_interlock()){
		heartbeat_send.base_mode|=MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
	}
	if(get_soft_armed()){
		heartbeat_send.base_mode|=MAV_MODE_FLAG_SAFETY_ARMED;
		if(m_Logger_Status==Logger_Idle){
			Logger_Enable();
		}
	}else{
		if(m_Logger_Status==Logger_Record){
			Logger_Disable();
		}
		get_mav_target=false;
	}
	if(m_Logger_Status==Logger_Record){
		heartbeat_send.base_mode|=MAV_MODE_FLAG_HIL_ENABLED;
	}
	if(get_gnss_state()){
		heartbeat_send.base_mode|=MAV_MODE_FLAG_GUIDED_ENABLED;
	}
	mavlink_msg_heartbeat_encode(mavlink_system.sysid, mavlink_system.compid, &msg_heartbeat, &heartbeat_send);

	//电量
	battery_status.type=2;//3s锂电池
	battery_status.temperature=(int16_t)(get_baro_temp()*100);
	battery_status.voltages[0]=(uint16_t)(get_5v_in()*1000);
	battery_status.voltages[1]=(uint16_t)(get_batt_volt()*1000);
	battery_status.voltages[2]=(uint16_t)uwb->Anchordistance[0];
	battery_status.voltages[3]=(uint16_t)uwb->Anchordistance[1];
	battery_status.voltages[4]=(uint16_t)uwb->Anchordistance[2];
	battery_status.voltages[5]=(uint16_t)uwb->Anchordistance[3];
	battery_status.voltages[6]=uwb->Anchorvolt[0];
	battery_status.voltages[7]=uwb->Anchorvolt[1];
	battery_status.voltages[8]=uwb->Anchorvolt[2];
	battery_status.voltages[9]=uwb->Anchorvolt[3];
	battery_status.current_battery=(int16_t)(get_batt_current()*100);
	mavlink_msg_battery_status_encode(mavlink_system.sysid, mavlink_system.compid, &msg_battery_status, &battery_status);

#if COMM_0==MAV_COMM
		mavlink_send_buffer(MAVLINK_COMM_0, &msg_heartbeat);
		if(gcs_channel==MAVLINK_COMM_0||offboard_channel==MAVLINK_COMM_0){
			mavlink_send_buffer(MAVLINK_COMM_0, &msg_battery_status);
		}
#endif
#if COMM_1==MAV_COMM
		mavlink_send_buffer(MAVLINK_COMM_1, &msg_heartbeat);
		if(gcs_channel==MAVLINK_COMM_1||offboard_channel==MAVLINK_COMM_1){
			mavlink_send_buffer(MAVLINK_COMM_1, &msg_battery_status);
		}
#endif
#if COMM_2==MAV_COMM
		mavlink_send_buffer(MAVLINK_COMM_2, &msg_heartbeat);
		if(gcs_channel==MAVLINK_COMM_2||offboard_channel==MAVLINK_COMM_2){
			mavlink_send_buffer(MAVLINK_COMM_2, &msg_battery_status);
		}
#endif
#if COMM_3==MAV_COMM
		mavlink_send_buffer(MAVLINK_COMM_3, &msg_heartbeat);
		if(gcs_channel==MAVLINK_COMM_3||offboard_channel==MAVLINK_COMM_3){
			mavlink_send_buffer(MAVLINK_COMM_3, &msg_battery_status);
		}
#endif
#if COMM_4==MAV_COMM
		mavlink_send_buffer(MAVLINK_COMM_4, &msg_heartbeat);
		if(gcs_channel==MAVLINK_COMM_4||offboard_channel==MAVLINK_COMM_4){
			mavlink_send_buffer(MAVLINK_COMM_4, &msg_battery_status);
		}
#endif
#if COMM_UWB==MAV_COMM
		uwb_send_mavlink_buffer(&msg_heartbeat);
		uwb_send_mavlink_buffer(&msg_battery_status);
#endif
}

void send_mavlink_mission_ack(mavlink_channel_t chan, MAV_MISSION_RESULT result){
	mavlink_message_t msg_mission_ack;
	mavlink_mission_ack_t mission_ack;
	mission_ack.type=result;
	mavlink_msg_mission_ack_encode(mavlink_system.sysid, mavlink_system.compid, &msg_mission_ack, &mission_ack);
	mavlink_send_buffer(chan, &msg_mission_ack);
}

void send_mavlink_mission_item_reached(mavlink_channel_t chan, uint16_t seq){
	mavlink_message_t msg_mission_item_reached;
	mavlink_mission_item_reached_t mission_item_reached;
	mission_item_reached.seq=seq;
	mavlink_msg_mission_item_reached_encode(mavlink_system.sysid, mavlink_system.compid, &msg_mission_item_reached, &mission_item_reached);
	mavlink_send_buffer(chan, &msg_mission_item_reached);
}

void send_mavlink_mission_count(mavlink_channel_t chan){
	mission_count_send.count=gnss_point_num;
	mavlink_msg_mission_count_encode(mavlink_system.sysid, mavlink_system.compid, &msg_mission_count, &mission_count_send);
	mavlink_send_buffer(chan, &msg_mission_count);
}

void send_mavlink_mission_list(mavlink_channel_t chan){
	for(uint16_t i=0;i<gnss_point_num;i++){
		mission_item_send.seq=i;
		mission_item_send.x=gnss_point_prt[i].x;
		mission_item_send.y=gnss_point_prt[i].y;
		mission_item_send.z=gnss_point_prt[i].z;
		mavlink_msg_mission_item_encode(mavlink_system.sysid, mavlink_system.compid, &msg_mission_item, &mission_item_send);
		mavlink_send_buffer(chan, &msg_mission_item);
	}
}

void send_mavlink_commond_ack(mavlink_channel_t chan, MAV_CMD mav_cmd, MAV_CMD_ACK result){
	mavlink_message_t msg_command_ack;
	mavlink_command_ack_t command_ack;
	command_ack.command=mav_cmd;
	command_ack.result=result;
	mavlink_msg_command_ack_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_ack, &command_ack);
	mavlink_send_buffer(chan, &msg_command_ack);
}

static uint8_t accel_cali_num=0;
static uint32_t takeoff_time=0;
void send_mavlink_data(mavlink_channel_t chan)
{
	uint32_t time=HAL_GetTick();
	if((time-time_last_heartbeat[(uint8_t)chan])>5000&&(HeartBeatFlags&(EVENTBIT_HEARTBEAT_COMM_0<<(uint8_t)chan))){
		HeartBeatFlags&=(0xFF^(EVENTBIT_HEARTBEAT_COMM_0<<(uint8_t)chan));
		if(chan==gcs_channel){
			gcs_connected=false;
			reset_rc_channels();
			set_rc_channels_override(false);
			gcs_channel=255;
			if(chan==MAVLINK_COMM_0){
				offboard_connected=false;
				offboard_channel=255;
			}
		}else if(chan==offboard_channel){
			offboard_connected=false;
			offboard_channel=255;
		}
		return;
	}

	//姿态+位置
	global_attitude_position.pitch=ahrs_pitch_rad();
	global_attitude_position.roll=ahrs_roll_rad();
	global_attitude_position.yaw=ahrs_yaw_rad();
	if(use_uwb){
		global_attitude_position.x=get_pos_x()*cosf(uwb_yaw_delta)-get_pos_y()*sinf(uwb_yaw_delta);
		global_attitude_position.y=get_pos_x()*sinf(uwb_yaw_delta)+get_pos_y()*cosf(uwb_yaw_delta);
	}else{
		global_attitude_position.x=get_pos_x();
		global_attitude_position.y=get_pos_y();
	}
	global_attitude_position.z=get_pos_z();
	global_attitude_position.usec=time;
	mavlink_msg_global_vision_position_estimate_encode(mavlink_system.sysid, mavlink_system.compid, &msg_global_attitude_position, &global_attitude_position);
	mavlink_send_buffer(chan, &msg_global_attitude_position);

	//经纬高+速度
	if(use_uwb){
		global_position_int.lat=(int32_t)(uwb_pos.x*cosf(uwb_yaw_delta)-uwb_pos.y*sinf(uwb_yaw_delta));//cm
		global_position_int.lon=(int32_t)(uwb_pos.x*sinf(uwb_yaw_delta)+uwb_pos.y*cosf(uwb_yaw_delta));//cm
		global_position_int.alt=(int32_t)uwb_pos.z;//cm
	}else{
		if((USE_ODOMETRY&&odom_2d==0)){
			global_position_int.lat=-1;
			global_position_int.lon=-1;
		}else{
			global_position_int.lat=gps_position->lat;//deg*1e7
			global_position_int.lon=gps_position->lon;//deg*1e7
		}
		global_position_int.alt=gps_position->alt;//mm
	}
	global_position_int.relative_alt=(int32_t)(rangefinder_state.alt_cm*10);//对地高度 mm
	if(get_gnss_state()){
		global_position_int.hdg=(uint16_t)gps_position->satellites_used|((uint16_t)gps_position->heading_status<<8)|((uint16_t)(gps_position->fix_type+get_gnss_stabilize())<<12);//卫星数+定向状态+定位状态
	}else{
		global_position_int.hdg=0;
	}

	global_position_int.vx=get_vel_x(); //速度cm/s
	global_position_int.vy=get_vel_y(); //速度cm/s
	global_position_int.vz=get_vel_z(); //速度cm/s
	if(takeoff_time>0){
		global_position_int.time_boot_ms=time-takeoff_time;//起飞时间 ms
	}else{
		global_position_int.time_boot_ms=0;
	}
	mavlink_msg_global_position_int_encode(mavlink_system.sysid, mavlink_system.compid, &msg_global_position_int, &global_position_int);
	mavlink_send_buffer(chan, &msg_global_position_int);

	if((uwb->TAG_ID==1||offboard_connected)&&param->uwb_tag_max.value>1&&uwb->get_range_distance(1,2)>0){
		for(uint8_t i=1;i<param->uwb_tag_max.value;i++){
			for(uint8_t j=i+1;j<=param->uwb_tag_max.value;j++){
				command_long.command=MAV_CMD_CONDITION_DISTANCE;
				command_long.param1=(float)i;
				command_long.param2=(float)j;
				command_long.param3=(float)uwb->get_range_distance(i,j);//cm
				mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
				mavlink_send_buffer(chan, &msg_command_long);
			}
		}
	}

	if(chan==gcs_channel){
		//加速度计校准
		if(!accel_cal_succeed){
			accel_cali_num=0;
			command_long.command=MAV_CMD_PREFLIGHT_CALIBRATION;
			command_long.param1=1.0f;
			command_long.param2=(float)accelCalibrator->get_num_samples_collected()+1;
			mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
			mavlink_send_buffer(chan, &msg_command_long);
		}else{
			if((accelCalibrator->get_num_samples_collected()==ACCELCAL_VEHICLE_POS_BACK)&&(accel_cali_num<50)){
				command_long.command=MAV_CMD_PREFLIGHT_CALIBRATION;
				command_long.param1=1.0f;
				command_long.param2=(float)accelCalibrator->get_num_samples_collected()+1;
				mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
				mavlink_send_buffer(chan, &msg_command_long);
				accel_cali_num++;
			}
		}

		//罗盘校准
		if(mag_correcting){
			command_long.command=MAV_CMD_PREFLIGHT_CALIBRATION;
			command_long.param1=2.0f;
			command_long.param2=completion_percent;
			mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
			mavlink_send_buffer(chan, &msg_command_long);
		}else{
			if(compass_cal_succeed){
				command_long.command=MAV_CMD_PREFLIGHT_CALIBRATION;
				command_long.param1=2.0f;
				command_long.param2=2.0f;
				mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
				mavlink_send_buffer(chan, &msg_command_long);
				compass_cal_succeed=false;
			}
		}

		//电脑端地面站需要显示遥控信号
		if(rc_channels_sendback){
			rc_channels_t.chan1_raw=input_channel_roll();
			rc_channels_t.chan2_raw=input_channel_pitch();
			rc_channels_t.chan3_raw=input_channel_throttle();
			rc_channels_t.chan4_raw=input_channel_yaw();
			rc_channels_t.chan5_raw=input_channel_5();
			rc_channels_t.chan6_raw=input_channel_6();
			rc_channels_t.chan7_raw=input_channel_7();
			rc_channels_t.chan8_raw=input_channel_8();
			rc_channels_t.chan9_raw=input_channel_9();
			rc_channels_t.chan10_raw=input_channel_10();
			rc_channels_t.chan11_raw=input_channel_11();
			rc_channels_t.chan12_raw=input_channel_12();
			rc_channels_t.chan13_raw=input_channel_13();
			rc_channels_t.chan14_raw=input_channel_14();
			rc_channels_t.chancount=RC_INPUT_CHANNELS;
			rc_channels_t.rssi=254;
			mavlink_msg_rc_channels_encode(mavlink_system.sysid, mavlink_system.compid, &msg_rc_channels, &rc_channels_t);
			mavlink_send_buffer(chan, &msg_rc_channels);
		}
	}
}

void distribute_mavlink_data(void){
#if COMM_0==MAV_COMM
	if (HeartBeatFlags&EVENTBIT_HEARTBEAT_COMM_0){
		send_mavlink_data(MAVLINK_COMM_0);
	}
#endif
#if COMM_1==MAV_COMM
	if (HeartBeatFlags&EVENTBIT_HEARTBEAT_COMM_1){
		send_mavlink_data(MAVLINK_COMM_1);
	}
#endif
#if COMM_2==MAV_COMM
	if (HeartBeatFlags&EVENTBIT_HEARTBEAT_COMM_2){
		send_mavlink_data(MAVLINK_COMM_2);
	}
#endif
#if COMM_3==MAV_COMM
	if (HeartBeatFlags&EVENTBIT_HEARTBEAT_COMM_3){
		send_mavlink_data(MAVLINK_COMM_3);
	}
#endif
#if COMM_4==MAV_COMM
	if (HeartBeatFlags&EVENTBIT_HEARTBEAT_COMM_4){
		send_mavlink_data(MAVLINK_COMM_4);
	}
#endif
}

static mavlink_message_t msg_received[MAVLINK_COMM_NUM_BUFFERS];
static mavlink_status_t status[MAVLINK_COMM_NUM_BUFFERS];
void comm0_callback(void){
#if COMM_0==MAV_COMM
	parse_mavlink_data(MAVLINK_COMM_0, get_comm0_data(), &msg_received[MAVLINK_COMM_0], &status[MAVLINK_COMM_0]);
#endif
}

void comm1_callback(void){
#if COMM_1==MAV_COMM
	parse_mavlink_data(MAVLINK_COMM_1, get_comm1_data(), &msg_received[MAVLINK_COMM_1], &status[MAVLINK_COMM_1]);
#elif COMM_1==GPS_COMM
	get_gnss_data(get_comm1_data());
#elif COMM_1==TFMINI_COMM
	get_tfmini_data(get_comm1_data());
#elif COMM_1==LC302_COMM
	get_lc302_data(get_comm1_data());
#elif COMM_1==TF2MINI_COMM
	get_uart_tf2mini_data(get_comm1_data());
#endif
}

void comm2_callback(void){
#if COMM_2==MAV_COMM
	parse_mavlink_data(MAVLINK_COMM_2, get_comm2_data(), &msg_received[MAVLINK_COMM_2], &status[MAVLINK_COMM_2]);
#elif COMM_2==GPS_COMM
	get_gnss_data(get_comm2_data());
#elif COMM_2==TFMINI_COMM
	get_tfmini_data(get_comm2_data());
#elif COMM_2==LC302_COMM
	get_lc302_data(get_comm2_data());
#elif COMM_2==TF2MINI_COMM
	get_uart_tf2mini_data(get_comm2_data());
#endif
}

void comm3_callback(void){
#if COMM_3==MAV_COMM
	parse_mavlink_data(MAVLINK_COMM_3, get_comm3_data(), &msg_received[MAVLINK_COMM_3], &status[MAVLINK_COMM_3]);
#elif COMM_3==GPS_COMM
	get_gnss_data(get_comm3_data());
#elif COMM_3==TFMINI_COMM
	get_tfmini_data(get_comm3_data());
#elif COMM_3==LC302_COMM
	get_lc302_data(get_comm3_data());
#elif COMM_3==TF2MINI_COMM
	get_uart_tf2mini_data(get_comm3_data());
#endif
}

void comm4_callback(void){
#if COMM_4==MAV_COMM
	parse_mavlink_data(MAVLINK_COMM_4, get_comm4_data(), &msg_received[MAVLINK_COMM_4], &status[MAVLINK_COMM_4]);
#elif COMM_4==GPS_COMM
	get_gnss_data(get_comm4_data());
#elif COMM_4==TFMINI_COMM
	get_tfmini_data(get_comm4_data());
#elif COMM_4==LC302_COMM
	get_lc302_data(get_comm4_data());
#elif COMM_4==TF2MINI_COMM
	get_uart_tf2mini_data(get_comm4_data());
#endif
}

void comm_uwb_callback(void){
#if COMM_UWB==MAV_COMM
	uint16_t length=rbGetCount(&uwb->ringbuffer_uwb_rx);
	while(length>0){
	  length--;
	  parse_mavlink_data(MAVLINK_COMM_5, rbPop(&uwb->ringbuffer_uwb_rx), &msg_received[MAVLINK_COMM_5], &status[MAVLINK_COMM_5]);
	}
#elif COMM_UWB==CONFIG_COMM
	if(COMM_0==DEV_COMM){
		uint16_t length=rbGetCount(&uwb->ringbuffer_uwb_rx);
		while(length>0){
		  length--;
		  set_comm0_data(rbPop(&uwb->ringbuffer_uwb_rx));
		}
		length=get_comm0_available();
		while(length>0){
		  length--;
		  rbPush(&uwb->ringbuffer_uwb_tx, get_comm0_data());
		}
	}
#endif
}

void uwb_send_mavlink_buffer(mavlink_message_t *msg)
{
	uint16_t len=MAVLINK_NUM_NON_PAYLOAD_BYTES + (uint16_t)msg->len;
	uwb->send_buff((uint8_t*)&msg->magic, len);
}

void send_mavlink_param_list(mavlink_channel_t chan)
{
	command_long.command=MAV_CMD_DO_SET_PARAMETER;

	command_long.param1=1.0f;
	command_long.param2=param->angle_roll_p.value;
	mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
	mavlink_send_buffer(chan, &msg_command_long);

	command_long.param1=2.0f;
	command_long.param2=param->angle_pitch_p.value;
	mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
	mavlink_send_buffer(chan, &msg_command_long);

	command_long.param1=3.0f;
	command_long.param2=param->angle_yaw_p.value;
	mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
	mavlink_send_buffer(chan, &msg_command_long);

	command_long.param1=4.0f;
	command_long.param2=param->rate_roll_pid.value_p;
	command_long.param3=param->rate_roll_pid.value_i;
	command_long.param4=param->rate_roll_pid.value_d;
	command_long.param5=param->rate_roll_pid.value_imax;
	command_long.param6=param->rate_roll_pid.value_filt_hz;
	mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
	mavlink_send_buffer(chan, &msg_command_long);

	command_long.param1=5.0f;
	command_long.param2=param->rate_pitch_pid.value_p;
	command_long.param3=param->rate_pitch_pid.value_i;
	command_long.param4=param->rate_pitch_pid.value_d;
	command_long.param5=param->rate_pitch_pid.value_imax;
	command_long.param6=param->rate_pitch_pid.value_filt_hz;
	mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
	mavlink_send_buffer(chan, &msg_command_long);

	command_long.param1=6.0f;
	command_long.param2=param->rate_yaw_pid.value_p;
	command_long.param3=param->rate_yaw_pid.value_i;
	command_long.param4=param->rate_yaw_pid.value_d;
	command_long.param5=param->rate_yaw_pid.value_imax;
	command_long.param6=param->rate_yaw_pid.value_filt_hz;
	mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
	mavlink_send_buffer(chan, &msg_command_long);

	command_long.param1=7.0f;
	command_long.param2=param->pos_z_p.value;
	mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
	mavlink_send_buffer(chan, &msg_command_long);

	command_long.param1=8.0f;
	command_long.param2=param->vel_z_p.value;
	mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
	mavlink_send_buffer(chan, &msg_command_long);

	command_long.param1=9.0f;
	command_long.param2=param->accel_z_pid.value_p;
	command_long.param3=param->accel_z_pid.value_i;
	command_long.param4=param->accel_z_pid.value_d;
	command_long.param5=param->accel_z_pid.value_imax;
	command_long.param6=param->accel_z_pid.value_filt_hz;
	mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
	mavlink_send_buffer(chan, &msg_command_long);

	command_long.param1=10.0f;
	command_long.param2=param->pos_xy_p.value;
	mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
	mavlink_send_buffer(chan, &msg_command_long);

	command_long.param1=11.0f;
	command_long.param2=param->vel_xy_pid.value_p;
	command_long.param3=param->vel_xy_pid.value_i;
	command_long.param4=param->vel_xy_pid.value_d;
	command_long.param5=param->vel_xy_pid.value_imax;
	command_long.param6=param->vel_xy_pid.value_filt_hz;
	command_long.param7=param->vel_xy_pid.value_filt_d_hz;
	mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
	mavlink_send_buffer(chan, &msg_command_long);

	command_long.param1=12.0f;
	command_long.param2=param->acro_y_expo.value;
	mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
	mavlink_send_buffer(chan, &msg_command_long);

	command_long.param1=13.0f;
	command_long.param2=param->acro_yaw_p.value;
	mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
	mavlink_send_buffer(chan, &msg_command_long);

	command_long.param1=14.0f;
	command_long.param2=param->throttle_midzone.value;
	mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
	mavlink_send_buffer(chan, &msg_command_long);

	command_long.param1=15.0f;
	command_long.param2=param->pilot_speed_dn.value;
	mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
	mavlink_send_buffer(chan, &msg_command_long);

	command_long.param1=16.0f;
	command_long.param2=param->pilot_speed_up.value;
	mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
	mavlink_send_buffer(chan, &msg_command_long);

	command_long.param1=17.0f;
	command_long.param2=param->rangefinder_gain.value;
	mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
	mavlink_send_buffer(chan, &msg_command_long);

	command_long.param1=18.0f;
	command_long.param2=param->angle_max.value;
	mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
	mavlink_send_buffer(chan, &msg_command_long);

	command_long.param1=19.0f;
	command_long.param2=param->pilot_accel_z.value;
	mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
	mavlink_send_buffer(chan, &msg_command_long);

	command_long.param1=20.0f;
	command_long.param2=param->pilot_takeoff_alt.value;
	mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
	mavlink_send_buffer(chan, &msg_command_long);

	command_long.param1=21.0f;
	command_long.param2=param->spool_up_time.value;
	mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
	mavlink_send_buffer(chan, &msg_command_long);

	command_long.param1=22.0f;
	command_long.param2=param->throttle_filt.value;
	mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
	mavlink_send_buffer(chan, &msg_command_long);

	command_long.param1=23.0f;
	command_long.param2=param->t_hover_update_min.value;
	mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
	mavlink_send_buffer(chan, &msg_command_long);

	command_long.param1=24.0f;
	command_long.param2=param->t_hover_update_max.value;
	mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
	mavlink_send_buffer(chan, &msg_command_long);

	command_long.param1=25.0f;
	command_long.param2=param->vib_land.value;
	mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
	mavlink_send_buffer(chan, &msg_command_long);

	command_long.param1=26.0f;
	command_long.param2=param->auto_land_speed.value;
	mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
	mavlink_send_buffer(chan, &msg_command_long);

	command_long.param1=27.0f;
	command_long.param2=param->lowbatt_return_volt.value;
	mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
	mavlink_send_buffer(chan, &msg_command_long);

	command_long.param1=28.0f;
	command_long.param2=param->lowbatt_land_volt.value;
	mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
	mavlink_send_buffer(chan, &msg_command_long);

	command_long.param1=29.0f;
	command_long.param2=param->poshold_vel_max.value;
	mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
	mavlink_send_buffer(chan, &msg_command_long);

	command_long.param1=30.0f;
	command_long.param2=param->poshold_accel_max.value;
	mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
	mavlink_send_buffer(chan, &msg_command_long);

	command_long.param1=31.0f;
	command_long.param2=param->mission_vel_max.value;
	mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
	mavlink_send_buffer(chan, &msg_command_long);

	command_long.param1=32.0f;
	command_long.param2=param->mission_accel_max.value;
	mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
	mavlink_send_buffer(chan, &msg_command_long);

	command_long.param1=33.0f;
	command_long.param2=param->alt_return.value;
	mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
	mavlink_send_buffer(chan, &msg_command_long);

	command_long.param1=34.0f;
	command_long.param2=param->voltage_gain.value;
	mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
	mavlink_send_buffer(chan, &msg_command_long);

	command_long.param1=35.0f;
	command_long.param2=param->current_gain.value;
	mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
	mavlink_send_buffer(chan, &msg_command_long);

	command_long.param1=36.0f;
	command_long.param2=param->uwb_yaw_delta_deg.value;
	mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
	mavlink_send_buffer(chan, &msg_command_long);

	command_long.param1=37.0f;
	command_long.param2=(float)param->uwb_tag_id.value;
	mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
	mavlink_send_buffer(chan, &msg_command_long);

	command_long.param1=38.0f;
	command_long.param2=(float)param->uwb_tag_max.value;
	mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
	mavlink_send_buffer(chan, &msg_command_long);

	command_long.param1=39.0f;
	command_long.param2=param->uwb_anchor01_pos.value.x;
	command_long.param3=param->uwb_anchor01_pos.value.y;
	command_long.param4=param->uwb_anchor01_pos.value.z;
	mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
	mavlink_send_buffer(chan, &msg_command_long);

	command_long.param1=40.0f;
	command_long.param2=param->uwb_anchor02_pos.value.x;
	command_long.param3=param->uwb_anchor02_pos.value.y;
	command_long.param4=param->uwb_anchor02_pos.value.z;
	mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
	mavlink_send_buffer(chan, &msg_command_long);

	command_long.param1=41.0f;
	command_long.param2=param->uwb_anchor03_pos.value.x;
	command_long.param3=param->uwb_anchor03_pos.value.y;
	command_long.param4=param->uwb_anchor03_pos.value.z;
	mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
	mavlink_send_buffer(chan, &msg_command_long);

	command_long.param1=42.0f;
	command_long.param2=param->uwb_anchor04_pos.value.x;
	command_long.param3=param->uwb_anchor04_pos.value.y;
	command_long.param4=param->uwb_anchor04_pos.value.z;
	mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
	mavlink_send_buffer(chan, &msg_command_long);

	command_long.param1=43.0f;
	command_long.param2=param->auto_takeoff_speed.value;
	mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
	mavlink_send_buffer(chan, &msg_command_long);

	command_long.param1=44.0f;
	command_long.param2=param->landing_lock_alt.value;
	mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
	mavlink_send_buffer(chan, &msg_command_long);

	command_long.param1=45.0f;
	command_long.param2=param->flow_gain.value.x;
	command_long.param3=param->flow_gain.value.y;
	command_long.param4=param->flow_gain.value.z;
	mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
	mavlink_send_buffer(chan, &msg_command_long);

	command_long.param1=46.0f;
	command_long.param2=param->gnss_offset.value.x;
	command_long.param3=param->gnss_offset.value.y;
	command_long.param4=param->gnss_offset.value.z;
	mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
	mavlink_send_buffer(chan, &msg_command_long);

	command_long.param1=47.0f;
	command_long.param2=(float)param->comm1_bandrate.value;//波特率
	mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
	mavlink_send_buffer(chan, &msg_command_long);

	command_long.param1=48.0f;
	command_long.param2=(float)param->comm2_bandrate.value;//波特率
	mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
	mavlink_send_buffer(chan, &msg_command_long);

	command_long.param1=49.0f;
	command_long.param2=(float)param->comm3_bandrate.value;//波特率
	mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
	mavlink_send_buffer(chan, &msg_command_long);

	command_long.param1=50.0f;
	command_long.param2=(float)param->comm4_bandrate.value;//波特率
	mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
	mavlink_send_buffer(chan, &msg_command_long);
	/* *************************************************
	 * ****************Dev code begin*******************/
	// Warning! Developer can add your new code here!
	/* Demo
	 * 刷新参数列表
	command_long.param1=1001.0f;
	command_long.param2=param->demo_param_1.value.x;
	command_long.param3=param->demo_param_1.value.y;
	command_long.param4=param->demo_param_1.value.z;
	mavlink_msg_command_long_encode(mavlink_system.sysid, mavlink_system.compid, &msg_command_long, &command_long);
	mavlink_send_buffer(chan, &msg_command_long);
	 * */

	/* ****************Dev code end*********************
	 * *************************************************/
}

void ekf_z_reset(void){
	ekf_baro->reset();
	_baro_alt_filter.reset(0.0f);
	update_baro_alt();
	ekf_baro_alt();
	pos_control->set_alt_target_to_current_alt();
}

void ekf_xy_reset(void){
	ekf_odometry->reset();
	ekf_odom_xy();
	ekf_gnss->reset();
	ekf_gnss_xy();
	pos_control->set_xy_target(get_pos_x(), get_pos_y());
}

extern float *rc_range_min,*rc_range_max;
void rc_range_init(void){
	rc_range_min[0]=(float)param->channel_range.channel[0];
	rc_range_min[1]=(float)param->channel_range.channel[1];
	rc_range_min[2]=(float)param->channel_range.channel[2];
	rc_range_min[3]=(float)param->channel_range.channel[3];
	rc_range_max[0]=(float)param->channel_range.channel[4];
	rc_range_max[1]=(float)param->channel_range.channel[5];
	rc_range_max[2]=(float)param->channel_range.channel[6];
	rc_range_max[3]=(float)param->channel_range.channel[7];
	for(uint8_t i=0;i<4;i++){
		if(rc_range_min[i]<900||rc_range_min[i]>1100){
			rc_range_min[i]=1100;
		}
	}
	for(uint8_t i=0;i<4;i++){
		if(rc_range_max[i]<1900||rc_range_max[i]>2000){
			rc_range_max[i]=1900;
		}
	}
	rc_range_cal();
}

void motors_init(void){
	Motors::motor_frame_class frame_class;
	Motors::motor_frame_type frame_type;
	Motors::motor_pwm_type motor_type;
	switch(param->robot_type.value){
	case UAV_8_H:
		frame_class=Motors::MOTOR_FRAME_OCTAQUAD;
		frame_type=Motors::MOTOR_FRAME_TYPE_H;
		break;
	case UAV_4_X:
		frame_class=Motors::MOTOR_FRAME_QUAD;
		frame_type=Motors::MOTOR_FRAME_TYPE_X;
		break;
	case UGV_4:
		param->motor_type.value=BRUSH;
		break;
	case UGV_2:
		param->motor_type.value=BRUSH;
		break;
	case SPIDER_6:
		param->motor_type.value=SERVO;
		break;
	default://UAV_8_H
		frame_class=Motors::MOTOR_FRAME_OCTAQUAD;
		frame_type=Motors::MOTOR_FRAME_TYPE_H;
		break;
	}
	switch(param->motor_type.value){
	case ESC:
		motor_type=Motors::PWM_TYPE_ESC;
		break;
	case BRUSH:
		motor_type=Motors::PWM_TYPE_BRUSHED;
		break;
	case SERVO:
		motor_type=Motors::PWM_TYPE_SERVO;
		break;
	default:
		motor_type=Motors::PWM_TYPE_ESC;
		break;
	}
	motors->setup_motors(frame_class,frame_type,motor_type);
	motors->set_throttle_hover(constrain_float(motors->get_throttle_hover(), param->t_hover_update_min.value, param->t_hover_update_max.value));
	// disable output to motors and servos
	set_rcout_enable(false);
	FMU_PWM_Set_Output_Disable();
	motors->set_interlock(false);
	FMU_LED4_Control(true);
	FMU_LED7_Control(false);
}

void attitude_init(void){
	dcm_matrix_correct.from_euler(param->horizontal_correct.value.x, param->horizontal_correct.value.y, param->horizontal_correct.value.z);
	attitude->set_rotation_target_to_body(dcm_matrix_correct);
	attitude->get_angle_roll_p()(param->angle_roll_p.value);
	attitude->get_angle_pitch_p()(param->angle_pitch_p.value);
	attitude->get_angle_yaw_p()(param->angle_yaw_p.value);
	attitude->get_rate_roll_pid()(param->rate_roll_pid.value_p, param->rate_roll_pid.value_i,
			param->rate_roll_pid.value_d, param->rate_roll_pid.value_imax, param->rate_roll_pid.value_filt_hz, _dt);
	attitude->get_rate_pitch_pid()(param->rate_pitch_pid.value_p, param->rate_pitch_pid.value_i,
			param->rate_pitch_pid.value_d, param->rate_pitch_pid.value_imax, param->rate_pitch_pid.value_filt_hz, _dt);
	attitude->get_rate_yaw_pid()(param->rate_yaw_pid.value_p, param->rate_yaw_pid.value_i,
			param->rate_yaw_pid.value_d, param->rate_yaw_pid.value_imax, param->rate_yaw_pid.value_filt_hz, _dt);
	attitude->set_lean_angle_max(param->angle_max.value);
}

void pos_init(void){
	pos_control->get_pos_z_p()(param->pos_z_p.value);
	pos_control->get_vel_z_p()(param->vel_z_p.value);
	pos_control->get_accel_z_pid()(param->accel_z_pid.value_p, param->accel_z_pid.value_i,
			param->accel_z_pid.value_d, param->accel_z_pid.value_imax, param->accel_z_pid.value_filt_hz, _dt);
	pos_control->get_pos_xy_p()(param->pos_xy_p.value);
	pos_control->get_vel_xy_pid()(param->vel_xy_pid.value_p, param->vel_xy_pid.value_p, param->vel_xy_pid.value_i, param->vel_xy_pid.value_i, param->vel_xy_pid.value_d,
			param->vel_xy_pid.value_d, param->vel_xy_pid.value_imax, param->vel_xy_pid.value_filt_hz, param->vel_xy_pid.value_filt_d_hz, _dt);
	pos_control->set_dt(_dt);
	pos_control->init_xy_controller(true);
	pos_control->set_lean_angle_max_d(param->angle_max.value);
	Logger_Read_Gnss();
	rangefinder_state.alt_cm_filt.set_cutoff_frequency(rangefinder_filt_hz);//tfmini默认频率100hz
	_uwb_pos_filter.set_cutoff_frequency(uwb_pos_filt_hz);
	Baro_set_temp_offset_gain(param->baro_temp_offset_gain.value);
	usb_printf("baro-offset:%f\r\n",param->baro_temp_offset_gain.value);
}

bool uwb_init(void){
	if(uwb->uwb_init()){
		uwb->config_uwb(tag, param->uwb_tag_id.value, 1, 1, param->uwb_tag_max.value, 4);
		uwb_yaw_delta=-param->uwb_yaw_delta_deg.value*DEG_TO_RAD;
		usb_printf("tag id: %d\n",uwb->TAG_ID);
		uwb->set_anchor_positon(1, param->uwb_anchor01_pos.value.x, param->uwb_anchor01_pos.value.y, param->uwb_anchor01_pos.value.z);
		uwb->set_anchor_positon(2, param->uwb_anchor02_pos.value.x, param->uwb_anchor02_pos.value.y, param->uwb_anchor02_pos.value.z);
		uwb->set_anchor_positon(3, param->uwb_anchor03_pos.value.x, param->uwb_anchor03_pos.value.y, param->uwb_anchor03_pos.value.z);
		uwb->set_anchor_positon(4, param->uwb_anchor04_pos.value.x, param->uwb_anchor04_pos.value.y, param->uwb_anchor04_pos.value.z);
	}else{
		return false;
	}
	return true;
}

void update_accel_gyro_data(void){

	accel.x = icm20608_data.accf.x;//m/s/s
	accel.y = icm20608_data.accf.y;//m/s/s
	accel.z = icm20608_data.accf.z;//m/s/s

	if(isnan(accel.x) || isinf(accel.x) || isnan(accel.y) || isinf(accel.y) || isnan(accel.z) || isinf(accel.z)){
		return;
	}

	accel.rotate(ROTATION_YAW_270);
//	usb_printf("ax:%f,ay:%f,az:%f\n",accel.x,accel.y,accel.z);

	gyro.x = icm20608_data.gyrof.x;//rad/s
	gyro.y = icm20608_data.gyrof.y;//rad/s
	gyro.z = icm20608_data.gyrof.z;//rad/s

	if(isnan(gyro.x) || isinf(gyro.x) || isnan(gyro.y) || isinf(gyro.y) || isnan(gyro.z) || isinf(gyro.z)){
		return;
	}

	gyro.rotate(ROTATION_YAW_270);
//	usb_printf("gx:%f,gy:%f,gz:%f\n",gyro.x,gyro.y,gyro.z);

	Matrix3f accel_softiron{param->accel_diagonals.value.x,     param->accel_offdiagonals.value.x,  param->accel_offdiagonals.value.y,
							param->accel_offdiagonals.value.x,  param->accel_diagonals.value.y,     param->accel_offdiagonals.value.z,
							param->accel_offdiagonals.value.y,  param->accel_offdiagonals.value.z,  param->accel_diagonals.value.z};
	accel_correct=accel_softiron*(accel-param->accel_offsets.value);

	gyro_correct=gyro-gyro_offset;

	if(!initial_accel_gyro){
		if(accel_correct.z<-8&&accel_correct.length()<11&&accel_correct.length()>9){
			initial_accel_gyro=true;
			accel_filt=Vector3f(0,0,-GRAVITY_MSS);
			gyro_filt=Vector3f(0,0,0);
			_accel_filter.set_cutoff_frequency(400, accel_filt_hz);
			_gyro_filter.set_cutoff_frequency(400, gyro_filt_hz);
			_flow_gyro_filter.set_cutoff_frequency(400, flow_gyro_filt_hz);
			_accel_ef_filter.set_cutoff_frequency(400, accel_ef_filt_hz);
			ahrs_stage_compass=true;
		}
	}else{
		if(accel_correct.length()<50){//过滤奇异值
			accel_filt=_accel_filter.apply(accel_correct);
			gyro_filt=_gyro_filter.apply(gyro_correct);
			flow_gyro_offset=_flow_gyro_filter.apply(gyro_filt);
		}
	}
}

static Vector3f delta_velocity;
static uint8_t step;
static bool accel_calibrate(void){
	if(accel_cal_succeed){
		return true;
	}
	if(!initial_accel_cal){
		step=0;
		accelCalibrator->clear();
		accelCalibrator->start(ACCEL_CAL_AXIS_ALIGNED_ELLIPSOID, 6, 1.0f);
		initial_accel_cal=true;
	}

	switch(accelCalibrator->get_status()){
		case ACCEL_CAL_NOT_STARTED:
			step=0;
			return false;
		case ACCEL_CAL_WAITING_FOR_ORIENTATION:
			if(step!=accelCalibrator->get_num_samples_collected()+1){
				step=accelCalibrator->get_num_samples_collected()+1;
			}else{
				break;
			}
			switch (step) {
				case ACCELCAL_VEHICLE_POS_LEVEL:
					usb_printf("level\n");
					break;
				case ACCELCAL_VEHICLE_POS_LEFT:
					usb_printf("on its LEFT side\n");
					break;
				case ACCELCAL_VEHICLE_POS_RIGHT:
					usb_printf("on its RIGHT side\n");
					break;
				case ACCELCAL_VEHICLE_POS_NOSEDOWN:
					usb_printf("nose DOWN\n");
					break;
				case ACCELCAL_VEHICLE_POS_NOSEUP:
					usb_printf("nose UP\n");
					break;
				case ACCELCAL_VEHICLE_POS_BACK:
					usb_printf("on its BACK\n");
					break;
				default:
					usb_printf("get all samples\n");
					break;
			}
			break;
		case ACCEL_CAL_COLLECTING_SAMPLE:
			delta_velocity=accel*_dt;
			accelCalibrator->new_sample(delta_velocity,_dt);
			break;
		case ACCEL_CAL_SUCCESS:
			step=0;
			accel_cal_succeed=true;
			accelCalibrator->get_calibration(param->accel_offsets.value, param->accel_diagonals.value, param->accel_offdiagonals.value);
			dataflash->set_param_vector3f(param->accel_offsets.num, param->accel_offsets.value);
			dataflash->set_param_vector3f(param->accel_diagonals.num, param->accel_diagonals.value);
			dataflash->set_param_vector3f(param->accel_offdiagonals.num, param->accel_offdiagonals.value);
			usb_printf("accel calibrate succeed!\n");
			break;
		case ACCEL_CAL_FAILED:
			step=0;
			usb_printf("accel calibrate failed!\n");
			initial_accel_cal=false;
			accel_cal_succeed=false;
			break;
	}
	return accel_cal_succeed;
}

bool gyro_calibrate(void){
	if(gyro_cal_succeed){
		return true;
	}
	Vector3f last_average, best_avg;
	Vector3f new_gyro_offset;
	float best_diff;
	bool converged;

	// remove existing gyro offsets
	new_gyro_offset.zero();
	best_diff = -1.0f;
	last_average.zero();
	converged = false;

	// the strategy is to average 50 points over 0.5 seconds, then do it
	// again and see if the 2nd average is within a small margin of
	// the first

	// we try to get a good calibration estimate for up to 30 seconds
	// if the gyros are stable, we should get it in 1 second
	for (int16_t j = 0; j <= 30*2 ; j++) {
		Vector3f gyro_sum, gyro_avg, gyro_diff;
		Vector3f accel_start;
		float diff_norm=0;
		uint8_t i;

		gyro_sum.zero();
		accel_start = accel_filt;
		for (i=0; i<100; i++) {
			update_accel_gyro_data();
			gyro_sum += gyro;
			osDelay(5);
		}

		Vector3f accel_diff = accel_filt - accel_start;
		if (accel_diff.length() > 0.2f) {
			// the accelerometers changed during the gyro sum. Skip
			// this sample. This copes with doing gyro cal on a
			// steadily moving platform. The value 0.2 corresponds
			// with around 5 degrees/second of rotation.
			continue;
		}

		gyro_avg = gyro_sum / i;
		gyro_diff = last_average - gyro_avg;
		diff_norm = gyro_diff.length();

		if (best_diff < 0) {
			best_diff = diff_norm;
			best_avg = gyro_avg;
		} else if (gyro_diff.length() < radians(GYRO_INIT_MAX_DIFF_DPS)) {
			// we want the average to be within 0.1 bit, which is 0.04 degrees/s
			last_average = (gyro_avg * 0.5f) + (last_average * 0.5f);
			if (!converged || last_average.length() < new_gyro_offset.length()) {
				new_gyro_offset = last_average;
			}
			if (!converged) {
				converged = true;
			}
		} else if (diff_norm < best_diff) {
			best_diff = diff_norm;
			best_avg = (gyro_avg * 0.5f) + (last_average * 0.5f);
		}
		last_average = gyro_avg;
		// we've kept the user waiting long enough - use the best pair we
		// found so far
		if (!converged) {
			gyro_offset = best_avg;
			// flag calibration as failed for this gyro
			gyro_cal_succeed = false;
		} else {
			gyro_cal_succeed = true;
			gyro_offset = new_gyro_offset;
			break;
		}
	}
	return gyro_cal_succeed;
}

static uint16_t clear_mag_correct_delta=0;
static Vector3f mag_orin, mag_curr, mag_offset;
static float c_last=0.0f,c_gain=1.0f;
void update_mag_data(void){
#if USE_MAG
	mag.x = qmc5883_data.magf.x;
	mag.y = qmc5883_data.magf.y;
	mag.z = qmc5883_data.magf.z;

	if(isnan(mag.x) || isinf(mag.x) || isnan(mag.y) || isinf(mag.y) || isnan(mag.z) || isinf(mag.z)){
		return;
	}

	mag.rotate(ROTATION_YAW_270);

	Matrix3f mag_softiron{param->mag_diagonals.value.x,     param->mag_offdiagonals.value.x,  param->mag_offdiagonals.value.y,
						  param->mag_offdiagonals.value.x,  param->mag_diagonals.value.y,     param->mag_offdiagonals.value.z,
						  param->mag_offdiagonals.value.y,  param->mag_offdiagonals.value.z,  param->mag_diagonals.value.z};
	mag_correct=mag_softiron*(mag+param->mag_offsets.value);
//	usb_printf("mag:%f|%f|%f|%f|%f|%f\n",param->mag_diagonals.value.x,param->mag_diagonals.value.y,param->mag_diagonals.value.z,param->mag_offdiagonals.value.x,param->mag_offdiagonals.value.y,param->mag_offdiagonals.value.z);

	if(is_equal(param->mag_offsets.value.x,0.0f)||is_equal(param->mag_offsets.value.y,0.0f)||is_equal(param->mag_offsets.value.z,0.0f)){
		return;
	}

	if(Buzzer_get_ring_type()!=BUZZER_IDLE){
		return;
	}

	if(!initial_mag){
		mag_filt=mag_correct;
		_mag_filter.set_cutoff_frequency(100, mag_filt_hz);
		initial_mag=true;
	}else{
		if(robot_state==STATE_TAKEOFF||robot_state==STATE_LANDED){//起飞时禁用磁罗盘
			mag_corrected=false;
			clear_mag_correct_delta=800;
			c_last=0.0f;
			mag_offset.zero();
		}else if(robot_state==STATE_FLYING){
			if(clear_mag_correct_delta>0){
				mag_corrected=false;
				if(clear_mag_correct_delta<=200){//电流磁偏
					c_last+=get_batt_current()*0.005;
					mag_curr=dcm_matrix.transposed()*mag_orin;
					mag_offset.x+=(mag_filt.x-mag_curr.x)*0.005;
					mag_offset.y+=(mag_filt.y-mag_curr.y)*0.005;
					mag_offset.z+=(mag_filt.z-mag_curr.z)*0.005;
					if(clear_mag_correct_delta==1){
						mag_correct-=mag_offset;
						_mag_filter.reset(mag_correct);
					}
				}
				clear_mag_correct_delta--;
			}else{
				mag_corrected=true;
				if(c_last>3.0f){
					c_gain=get_batt_current()/c_last;
				}else{
					c_gain=1.0f;
				}
				mag_correct-=mag_offset*c_gain;
			}
		}else{
			mag_orin=dcm_matrix*mag_filt;
			mag_corrected=true;
		}
		mag_filt = _mag_filter.apply(mag_correct);
	}
#endif
}

void compass_calibrate(void){
	if(HAL_GetTick()<2500){
		return;
	}
	if(!initial_compass_cal){
		uint8_t compass_idx=1;
		/* if we got the ahrs, we should set _check_orientation true*/
		compassCalibrator->set_orientation(ROTATION_NONE, true, true, true);
		compassCalibrator->start(true, 2, COMPASS_OFFSETS_MAX_DEFAULT, compass_idx);
		initial_compass_cal=true;
	}
	compassCalibrator->new_sample(mag);
	compassCalibrator->update(calibrate_failure);
	completion_percent=compassCalibrator->get_completion_percent()/100.0f;
	if(is_equal(completion_percent,1.0f)){
		compassCalibrator->get_calibration(param->mag_offsets.value, param->mag_diagonals.value, param->mag_offdiagonals.value);
		dataflash->set_param_vector3f(param->mag_offsets.num, param->mag_offsets.value);
		dataflash->set_param_vector3f(param->mag_diagonals.num, param->mag_diagonals.value);
		dataflash->set_param_vector3f(param->mag_offdiagonals.num, param->mag_offdiagonals.value);
		compass_cal_succeed=true;
		mag_correcting=false;
		usb_printf("compass calibrate succeed!\n");
	}
}

static float roll_sum=0, pitch_sum=0;
static uint8_t horizon_correct_flag=0;
static float ax_body=0.0f, ay_body=0.0f;
void ahrs_update(void){
	if((!gyro_calibrate())||(!accel_calibrate())||(!initial_accel_gyro)){
		ahrs_healthy=false;
		return;
	}

	if(horizon_correct){
		horizon_correct_flag++;
		roll_sum+=roll_rad;
		pitch_sum+=pitch_rad;
		if(horizon_correct_flag==100){
			param->horizontal_correct.value.x=-roll_sum/100;
			param->horizontal_correct.value.y=-pitch_sum/100;
			param->horizontal_correct.value.z=0;
			dataflash->set_param_vector3f(param->horizontal_correct.num, param->horizontal_correct.value);
			dcm_matrix_correct.from_euler(param->horizontal_correct.value.x, param->horizontal_correct.value.y, param->horizontal_correct.value.z);
			attitude->set_rotation_target_to_body(dcm_matrix_correct);
			horizon_correct=false;
			roll_sum=0;
			pitch_sum=0;
			horizon_correct_flag=0;
		}
	}

	if(ahrs_stage_compass){
		ahrs->update(mag_corrected, get_mav_yaw);
		//由ahrs的四元数推出旋转矩阵用于控制
		ahrs->quaternion2.rotation_matrix(dcm_matrix);
		attitude->set_rotation_body_to_ned(dcm_matrix);
		attitude->get_rotation_target_to_ned().to_euler(&roll_log, &pitch_log, &yaw_log);
		gyro_ef=dcm_matrix*gyro_filt;
		accel_ef=dcm_matrix*accel_filt;
		accel_ef_filt=_accel_ef_filter.apply(accel_ef);

		dcm_matrix.to_euler(&roll_rad, &pitch_rad, &yaw_rad);
		roll_deg=roll_rad*RAD_TO_DEG;
		pitch_deg=pitch_rad*RAD_TO_DEG;
		yaw_deg=yaw_rad*RAD_TO_DEG;
		cos_roll=cosf(roll_rad);
		sin_roll=sinf(roll_rad);
		cos_pitch=cosf(pitch_rad);
		sin_pitch=sinf(pitch_rad);
		cos_yaw=cosf(yaw_rad);
		sin_yaw=sinf(yaw_rad);
		ax_body=0.5*ax_body+0.5*(accel_ef_filt.x*cos_yaw + accel_ef_filt.y*sin_yaw);
		ay_body=0.5*ay_body+0.5*(-accel_ef_filt.x*sin_yaw + accel_ef_filt.y*cos_yaw);
		if(mag_correcting){
#if USE_MAG
			compass_calibrate();
#endif
			ahrs_healthy=false;
		}else{
			ahrs_healthy=true;
		}
	}
}

static float baro_alt_filt=0,baro_alt_init=0,baro_alt_correct=0;
static uint16_t init_baro=0;
static float baro_alt=0.0f;
static uint32_t baro_sample_tick=0;
void update_baro_alt(void){
	if(init_baro<20){//前20点不要
		init_baro++;
		return;
	}
	if(is_equal(param->baro_temp_offset_gain.value, 0.0f)){
		if(HAL_GetTick()-baro_sample_tick<100){
			return;
		}
		baro_sample_tick=HAL_GetTick();
		init_baro++;
		if(init_baro>200){
			param->baro_temp_offset_gain.value=(spl06_data.baro_alt-spl06_data.baro_alt_init)/(spl06_data.temp-spl06_data.temp_init);
			dataflash->set_param_float(param->baro_temp_offset_gain.num, param->baro_temp_offset_gain.value);
			Baro_set_temp_offset_gain(param->baro_temp_offset_gain.value);
			Buzzer_set_ring_type(BUZZER_INITIALED);
			usb_printf("baro-offset:%f\r\n",param->baro_temp_offset_gain.value);
		}
		return;
	}

	baro_alt=spl06_data.baro_alt*100.0f;
	if(isnan(baro_alt) || isinf(baro_alt)){
		return;
	}
	if(fabs(baro_alt)>800000 || is_equal(baro_alt, 0.0f)){
		return;
	}
	if(init_baro<30){
		baro_alt_init+=baro_alt/10;
		init_baro++;
		return;
	}
	if(!initial_baro){
		_baro_alt_filter.set_cutoff_frequency(10, baro_filt_hz);
		initial_baro=true;
	}else{
		baro_alt-=baro_alt_init;
		ekf_baro->fusion(baro_alt, baro_alt_correct);
		baro_alt_filt = _baro_alt_filter.apply(baro_alt_correct);
		get_baro_alt_filt=true;
	}
}

float get_baroalt_filt(void){//cm
	return baro_alt_filt;
}

float get_baro_temp(void){//℃
	return spl06_data.temp;
}

void ekf_baro_alt(void){
	if((!ahrs->is_initialed())||(!initial_baro)||(!ahrs_healthy)){
		return;
	}
	ekf_baro->update(get_baro_alt_filt, get_baroalt_filt());
}

void ekf_rf_alt(void){
	if((!ahrs->is_initialed())||(!rangefinder_state.alt_healthy)||(!ahrs_healthy)){
		return;
	}
	ekf_rangefinder->update(get_rangefinder_data, get_rangefinder_alt());
}

static RTC_TimeTypeDef sTime;
static RTC_DateTypeDef sDate;
static float yaw_gnss_offset=0.0f;
static uint8_t yaw_gnss_flag=0;
static uint32_t gnss_last_update_time=0;
static Vector2f gnss_sample_2d, ned_sample_2d_last;
static uint8_t gnss_stabilize=0;
static Vector3f gnss_gyro_offset, ned_pos, ned_vel;
bool get_gnss_stabilize(void){
	return gnss_stabilize==10;
}
void gnss_update(void){
	if((HAL_GetTick()-get_gnss_update_ms())>1000){
		set_gnss_state(false);
		gnss_stabilize=0;
		return;
	}
	if(get_gnss_update_ms()==gnss_last_update_time){
		return;
	}else{
		gnss_last_update_time=get_gnss_update_ms();
	}
	if(get_gnss_state()){
		if(!initial_gnss&&USE_MAG){
			gnss_origin_pos.lat=gps_position->lat;//纬度:deg*1e-7
			gnss_origin_pos.lng=gps_position->lon;//经度:deg*1e-7
			gnss_origin_pos.alt=gps_position->alt/10;//海拔：cm
			ahrs->set_declination(radians(Declination::get_declination((float)gnss_origin_pos.lat*1e-7, (float)gnss_origin_pos.lng*1e-7)));
			initial_gnss=true;
		}
		if(gps_position->heading_status==4&&USE_MAG){
			if(yaw_gnss_flag>=20){
				yaw_gnss_offset=wrap_PI(gps_position->heading*DEG_TO_RAD-yaw_rad);
				if(fabsf(yaw_gnss_offset)>M_PI_2){
					yaw_gnss_offset=wrap_PI(yaw_gnss_offset+M_PI);
				}
				ahrs->set_declination(ahrs->get_declination()+yaw_gnss_offset);
				yaw_gnss_flag=0;
			}
			yaw_gnss_flag++;
		}
		sDate.Year=gps_position->year-1970;
		sDate.Month=gps_position->month;
		sDate.Date=gps_position->day;
		sTime.Hours=gps_position->hour+(gps_position->lon/1e7/15+1);
		sTime.Minutes=gps_position->min;
		sTime.Seconds=gps_position->sec;
		HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
		HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
		gnss_current_pos.lat=gps_position->lat;//纬度:deg*1e-7
		gnss_current_pos.lng=gps_position->lon;//经度:deg*1e-7
		gnss_current_pos.alt=gps_position->alt/10;   //cm
		ned_pos=location_3d_diff_NED(gnss_origin_pos, gnss_current_pos)*100;//cm
		ned_vel.x=gps_position->vel_n_m_s*100+constrain_float(gnss_gyro_offset.y*param->gnss_offset.value.x, -100.0f, 100.0f);//cm
		ned_vel.y=gps_position->vel_e_m_s*100-constrain_float(gnss_gyro_offset.x*param->gnss_offset.value.y, -100.0f, 100.0f);//cm
		ned_vel.z=gps_position->vel_d_m_s*100+constrain_float(gnss_gyro_offset.z*param->gnss_offset.value.z, -100.0f, 100.0f);//cm
//		usb_printf_dir("$%d %d;", (int16_t)(ned_current_vel.x), (int16_t)(-gnss_gyro_offset.y*3));
//		usb_printf_dir("$%d %d;", (int16_t)(ned_current_vel.y), (int16_t)(gnss_gyro_offset.x*25));
//		usb_printf_dir("$%d %d;", (int16_t)(ned_current_vel.x),(int16_t)(ned_current_vel.y));
		if(USE_MAG){
			ned_current_pos=ned_pos;
			ned_current_vel=ned_vel;
			get_gnss_location=true;
		}else{
			ned_current_pos.z=ned_pos.z;
			ned_current_vel.z=ned_vel.z;
		}
		//机体坐标系->大地坐标系
		gnss_gyro_offset.x=gyro_filt.x*ahrs_cos_yaw()-gyro_filt.y*ahrs_sin_yaw();
		gnss_gyro_offset.y=gyro_filt.x*ahrs_sin_yaw()+gyro_filt.y*ahrs_cos_yaw();
		gnss_gyro_offset.z=gyro_filt.z;
		if(safe_sqrt(ned_vel.x*ned_vel.x+ned_vel.y*ned_vel.y)<10.0f&&gps_position->alt_noise<10.0f){
			gnss_stabilize++;
		}else{
			gnss_stabilize=0;
		}
		if(gnss_stabilize>10){
			gnss_stabilize=10;
		}
//		usb_printf("ned_current_vel:%f\n",safe_sqrt(ned_current_vel.x*ned_current_vel.x+ned_current_vel.y*ned_current_vel.y));
	}else{
		gnss_stabilize=0;
	}
}

void uwb_update(void){
	uwb->uwb_update();
}

static uint32_t currunt_uwb_ms, last_uwb_ms = 0;
void uwb_position_update(void){
#if USE_UWB
	FMU_LED6_Control(uwb->get_uwb_position);
	if(!uwb->get_uwb_position){
		return;
	}
//	usb_printf_dir("$%d %d %d %d;", uwb->Anchordistance[0], uwb->Anchordistance[1], uwb->Anchordistance[2], uwb->Anchordistance[3]);
	uwb->get_uwb_position=false;
	uwb_pos.x=uwb->uwb_position.x*cosf(uwb_yaw_delta)+uwb->uwb_position.y*sinf(uwb_yaw_delta);
	uwb_pos.y=-uwb->uwb_position.x*sinf(uwb_yaw_delta)+uwb->uwb_position.y*cosf(uwb_yaw_delta);
	uwb_pos.z=uwb->uwb_position.z;
	currunt_uwb_ms=HAL_GetTick();
	if(ekf_baro->vel_2d<100.0f&&robot_state==STATE_FLYING){
		uwb_pos.x=constrain_float(uwb_pos.x, ned_current_pos.x-50.0f, ned_current_pos.x+50.0f);
		uwb_pos.y=constrain_float(uwb_pos.y, ned_current_pos.y-50.0f, ned_current_pos.y+50.0f);
	}
	uwb_pos = _uwb_pos_filter.apply(uwb_pos, (float)(currunt_uwb_ms-last_uwb_ms)/1000.0f);
	ned_current_pos.x=ned_current_pos.x*0.5+uwb_pos.x*0.5;
	ned_current_pos.y=ned_current_pos.y*0.5+uwb_pos.y*0.5;
	get_gnss_location=true;
	last_uwb_ms = currunt_uwb_ms;
	use_uwb=true;
#endif
}

static uint32_t update_odom_time=0;
static float odom_pos_x=0.0f, odom_pos_y=0.0f, odom_vel_x=0.0f, odom_vel_y=0.0f, odom_dt=0.0f;
static float odom_vel_offset_x=0.0f, odom_vel_offset_y=0.0f;
static float odom_vel_x_buff[400], odom_vel_y_buff[400];
static int16_t odom_vel_last_tick=0, odom_tc=0, odom_vel_tick=0;
static bool odom_vel_init=false;
static LowPassFilterFloat odom_vel_x_filter, odom_vel_y_filter;
void ekf_odom_xy(void){
#if USE_ODOMETRY
	if(!ahrs->is_initialed()||(!ahrs_healthy)){
		return;
	}
	if(!odom_vel_init){
		for(uint16_t i=0;i<400;i++){
			odom_vel_x_buff[i]=0.0f;
			odom_vel_y_buff[i]=0.0f;
		}
		odom_vel_x_filter.set_cutoff_frequency(400, 10);
		odom_vel_y_filter.set_cutoff_frequency(400, 10);
		odom_vel_init=true;
	}
	if(USE_VINS){
		odom_tc=400*constrain_float(vins_tc, 0.0, 1.0);
		odom_dt=constrain_float(vins_dt,0.1,0.2);
	}else{
		odom_tc=400*constrain_float(lidar_tc, 0.0, 1.0);
		odom_dt=constrain_float(lidar_dt,0.1,0.2);
	}
	if(odom_3d.x==0&&odom_3d.y==0){
		return;
	}
	if(odom_vel_tick==0){
		odom_vel_x_buff[odom_vel_tick]=odom_vel_x_buff[399]+get_accel_ef_filt().x*100.0*0.0025;
		odom_vel_y_buff[odom_vel_tick]=odom_vel_y_buff[399]+get_accel_ef_filt().y*100.0*0.0025;
	}else{
		odom_vel_x_buff[odom_vel_tick]=odom_vel_x_buff[odom_vel_tick-1]+get_accel_ef_filt().x*100.0*0.0025;
		odom_vel_y_buff[odom_vel_tick]=odom_vel_y_buff[odom_vel_tick-1]+get_accel_ef_filt().y*100.0*0.0025;
	}
	update_pos=true;
	if(get_odom_xy){
//		usb_printf("dt:%f|%ld",odom_dt,HAL_GetTick());
		update_odom_time=HAL_GetTick();
		odom_vel_last_tick=odom_vel_tick-odom_tc;
		if(odom_vel_last_tick<0){
			odom_vel_last_tick+=400;
		}
		odom_vel_x=odom_dx/odom_dt;
		odom_vel_y=odom_dy/odom_dt;
		if(USE_VINS){
			odom_pos_x=odom_3d.x+odom_vel_x*vins_tc;
			odom_pos_y=odom_3d.y+odom_vel_y*vins_tc;
		}else{
			odom_pos_x=odom_3d.x+odom_vel_x*lidar_tc;
			odom_pos_y=odom_3d.y+odom_vel_y*lidar_tc;
		}
		odom_vel_offset_x=odom_vel_x-odom_vel_x_buff[odom_vel_last_tick];
		odom_vel_offset_y=odom_vel_y-odom_vel_y_buff[odom_vel_last_tick];
		if(robot_state==STATE_FLYING||robot_state==STATE_TAKEOFF){
			odom_vel_offset_x=constrain_float(odom_vel_offset_x, -pos_control->get_accel_xy()*0.3, pos_control->get_accel_xy()*0.3);
			odom_vel_offset_y=constrain_float(odom_vel_offset_y, -pos_control->get_accel_xy()*0.3, pos_control->get_accel_xy()*0.3);
		}
		for(uint16_t i=0;i<=odom_tc;i++){
			if((i+odom_vel_last_tick)>=400){
				odom_vel_x_buff[i+odom_vel_last_tick-400]+=odom_vel_offset_x;
				odom_vel_y_buff[i+odom_vel_last_tick-400]+=odom_vel_offset_y;
			}else{
				odom_vel_x_buff[i+odom_vel_last_tick]+=odom_vel_offset_x;
				odom_vel_y_buff[i+odom_vel_last_tick]+=odom_vel_offset_y;
			}
		}
	}else{
		if(HAL_GetTick()-update_odom_time>1000&&robot_state==STATE_STOP){//未起飞且定位源失效
			ekf_odometry->reset();
			if(!USE_MOTION){
				update_pos=false;
			}
		}
	}
	odom_vel_x=odom_vel_x_filter.apply(odom_vel_x_buff[odom_vel_tick]);
	odom_vel_y=odom_vel_y_filter.apply(odom_vel_y_buff[odom_vel_tick]);
//	usb_printf("x:%f|y:%f, %f|%f, %f|%f\n",odom_pos_x,odom_pos_y, odom_vel_x,odom_vel_y, odom_vel_offset_x, odom_vel_offset_y);
	odom_vel_tick++;
	if(odom_vel_tick==400){
		odom_vel_tick=0;
	}
	if(!USE_GNSS){
		ekf_odometry->update(get_odom_xy,odom_pos_x,odom_pos_y);
	}else{
		get_odom_xy=false;
	}
#endif
}

static uint32_t update_gnss_time=0;
static int16_t flow_odom_tick=0;
void ekf_gnss_xy(void){
#if USE_GNSS
	if(!ahrs->is_initialed()||(!ahrs_healthy)){
		return;
	}
	if(get_odom_time>0&&HAL_GetTick()-get_odom_time>500&&!USE_MOTION){
		odom_safe=false;
		robot_state_desired=STATE_LANDED;
	}
	if(opticalflow_state.healthy&&rangefinder_state.alt_healthy&&rangefinder_state.alt_cm<150.0f){
		if(update_odom_xy&&enable_odom&&odom_safe&&!USE_MAG&&get_gnss_location){
	//		usb_printf("odom:%f|%d\n",odom_2d,odom_safe);
			if(odom_2d<=50.0f&&odom_2d>0.0f){
				if(USE_VINS){
					vins_tc=constrain_float(vins_tc, 0.0, 0.98);
					flow_odom_tick=flow_i_buff-1-vins_tc*50;
				}else{
					lidar_tc=constrain_float(lidar_tc, 0.0, 0.98);
					flow_odom_tick=flow_i_buff-1-lidar_tc*50;
				}
				if(flow_odom_tick<0){
					flow_odom_tick+=50;
				}
				ned_current_pos.x=odom_3d.x+(opticalflow_state.pos.x-ned_pos_x_buff[flow_odom_tick]);
				ned_current_pos.y=odom_3d.y+(opticalflow_state.pos.y-ned_pos_y_buff[flow_odom_tick]);
				update_odom_xy=false;
			}else if(odom_2d>50.0f&&get_soft_armed()&&!USE_MOTION){
				odom_safe=false;
				robot_state_desired=STATE_LANDED;
			}
		}
	}
	if(USE_ODOMETRY&&enable_odom&&!USE_MAG){
		if(odom_safe&&odom_2d<=50.0f&&odom_2d>0.0f){
			ned_current_pos.x=odom_pos_x;
			ned_current_pos.y=odom_pos_y;
			ned_current_vel.x=odom_vel_x;
			ned_current_vel.y=odom_vel_y;
			get_gnss_location=true;
		}else if(odom_2d>50.0f&&get_soft_armed()&&!USE_MOTION){
			odom_safe=false;
			robot_state_desired=STATE_LANDED;
		}
	}
	if(get_gnss_location){
		update_pos=true;
		update_gnss_time=HAL_GetTick();
	}else{
		if(HAL_GetTick()-update_gnss_time>1000&&robot_state==STATE_STOP){//未起飞且定位源失效
			ekf_wind->reset();
			ekf_gnss->reset();
			update_pos=false;
		}
	}
#if USE_WIND
	ekf_wind->update(get_gnss_location,get_ned_vel_x(),get_ned_vel_y());
#endif
	ekf_gnss->update(get_gnss_location,get_ned_pos_x(),get_ned_pos_y(),get_ned_vel_x(),get_ned_vel_y());
#endif
}

float get_pos_x(void){//cm
#if USE_GNSS
	return ekf_gnss->pos_x;
#elif USE_ODOMETRY
	return ekf_odometry->pos_x;
#endif
}

float get_pos_y(void){//cm
#if USE_GNSS
	return ekf_gnss->pos_y;
#elif USE_ODOMETRY
	return ekf_odometry->pos_y;
#endif
}

float get_pos_z(void){//cm
	return ekf_baro->pos_z;
}

float get_vel_x(void){//cm/s
#if USE_GNSS
	return ekf_gnss->vel_x;
#elif USE_ODOMETRY
	return ekf_odometry->vel_x;
#endif
}

float get_vel_y(void){//cm/s
#if USE_GNSS
	return ekf_gnss->vel_y;
#elif USE_ODOMETRY
	return ekf_odometry->vel_y;
#endif
}

float get_vel_z(void){//cm/s
	return ekf_baro->vel_z;
}

static uint8_t low_batt_flag=0;
void sdled_update(void){
	if(get_soft_armed()){
		FMU_LED3_Control(true);
	}else{
		FMU_LED3_Control(false);
	}
	osDelay(200);
	if(m_Logger_Status!=Logger_Record){
		FMU_LED3_Control(false);
	}
	if(robot_state==STATE_STOP&&get_batt_volt()>5.3f&&get_batt_volt()<param->lowbatt_land_volt.value){
		low_batt_flag++;
		if(low_batt_flag>=5){
			Buzzer_set_ring_type(BUZZER_ERROR);
			usb_printf("Warning: low power!\n");
		}
	}else{
		low_batt_flag=0;
	}
}

// get_pilot_desired_heading - transform pilot's yaw input into a
// desired yaw rate
// returns desired yaw rate in degrees per second
float get_pilot_desired_yaw_rate(float stick_angle)
{
    float yaw_request;
    float deadband_top = ROLL_PITCH_YAW_INPUT_MAX/10;
    float deadband_bottom = -ROLL_PITCH_YAW_INPUT_MAX/10;

    if(stick_angle<deadband_bottom){
    	stick_angle=-(stick_angle-deadband_bottom)/(-ROLL_PITCH_YAW_INPUT_MAX-deadband_bottom)*ROLL_PITCH_YAW_INPUT_MAX;
    }else if(stick_angle<=deadband_top){
    	stick_angle=0;
    }else{
    	stick_angle=(stick_angle-deadband_top)/(ROLL_PITCH_YAW_INPUT_MAX-deadband_top)*ROLL_PITCH_YAW_INPUT_MAX;
    }

    // calculate yaw rate request
    if (param->acro_y_expo.value <= 0) {
        yaw_request = stick_angle * param->acro_yaw_p.value;
    } else {
        // expo variables
        float y_in, y_in3, y_out;

        // range check expo
        if (param->acro_y_expo.value > 1.0f || param->acro_y_expo.value < 0.5f) {
        	param->acro_y_expo.value = 1.0f;
        }

        // yaw expo
        y_in = stick_angle/ROLL_PITCH_YAW_INPUT_MAX;
        y_in3 = y_in*y_in*y_in;
        y_out = (param->acro_y_expo.value * y_in3) + ((1.0f - param->acro_y_expo.value) * y_in);
        yaw_request = ROLL_PITCH_YAW_INPUT_MAX * y_out * param->acro_yaw_p.value;
    }
    // convert pilot input to the desired yaw rate
    return yaw_request;
}

/*************************************************************
 *  throttle control
 *************************************************************/
static bool _manual_throttle=true;
void set_manual_throttle(bool manual_throttle){_manual_throttle=manual_throttle;}

bool has_manual_throttle(void) { return _manual_throttle; }

// update estimated throttle required to hover (if necessary)
//  called at 100hz
static void update_throttle_hover(void)
{
    // if not armed or landed exit
    if (!motors->get_armed() || ap->land_complete) {
        return;
    }

    // do not update in manual throttle modes or Drift
    if (has_manual_throttle()) {
        return;
    }

    // get throttle output
    float throttle = motors->get_throttle();
    float climb_rate=get_vel_z();
    // calc average throttle if we are in a level hover
    if (throttle > param->t_hover_update_min.value && throttle < param->t_hover_update_max.value && abs(climb_rate) < 60.0f && abs(roll_deg) < 5.0f && abs(pitch_deg) < 5.0f) {
        // Can we set the time constant automatically
        motors->update_throttle_hover(0.01f);
    }
}

// set_throttle_takeoff - allows parents to tell throttle controller we are taking off so I terms can be cleared
void set_throttle_takeoff(void)
{
    // tell position controller to reset alt target and reset I terms
    pos_control->init_takeoff();
    odom_vel_init=false;
//    attitude->get_rate_roll_pid().set_integrator(param->rate_pid_integrator.value.x);
//	attitude->get_rate_pitch_pid().set_integrator(param->rate_pid_integrator.value.y);
//	attitude->get_rate_yaw_pid().set_integrator(param->rate_pid_integrator.value.z);
}

float get_throttle_mid(void){
	float throttle_mid=(motors->get_throttle_max()+motors->get_throttle_min())/2;
	return throttle_mid;
}

// transform pilot's manual throttle input to make hover throttle mid stick
// used only for manual throttle modes
// thr_mid should be in the range 0 to 1, e.g. we should use the _throttle_hover
// returns throttle output 0 to 1
float get_pilot_desired_throttle(float throttle_control, float thr_mid)
{
    if (thr_mid <= 0.0f) {
        thr_mid = motors->get_throttle_hover();
    }

    float mid_stick = get_throttle_mid();
    // protect against unlikely divide by zero
    if (mid_stick <= 0) {
        mid_stick = 0.5;
    }

    // ensure reasonable throttle values
    throttle_control = constrain_float(throttle_control,0.0f,1.0f);

    // calculate normalised throttle input
    float throttle_in;
    if (throttle_control < mid_stick) {
        // below the deadband
        throttle_in = throttle_control*0.5f/(float)mid_stick;
    }else if(throttle_control > mid_stick) {
        // above the deadband
        throttle_in = 0.5f + (throttle_control-mid_stick) * 0.5f / (float)(1.0-mid_stick);
    }else{
        // must be in the deadband
        throttle_in = 0.5f;
    }

    float expo = constrain_float(-(thr_mid-0.5)/0.375, -0.5f, 1.0f);
    // calculate the output throttle using the given expo function
    float throttle_out = throttle_in*(1.0f-expo) + expo*throttle_in*throttle_in*throttle_in;
    return throttle_out;
}

// get_pilot_desired_climb_rate - transform pilot's throttle input to climb rate in cm/s
// without any deadzone at the bottom
float get_pilot_desired_climb_rate(float throttle_control)
{
    float desired_rate = 0.0f;
    float mid_stick = get_throttle_mid();
    float deadband_top = mid_stick + param->throttle_midzone.value;
    float deadband_bottom = mid_stick - param->throttle_midzone.value;

    // ensure a reasonable throttle value
    throttle_control = constrain_float(throttle_control,0.0f,1.0f);

    // ensure a reasonable deadzone
    param->throttle_midzone.value = constrain_float(param->throttle_midzone.value, 0, 0.3);

    // check throttle is above, below or in the deadband
    if (throttle_control < deadband_bottom) {
        // below the deadband
        desired_rate = param->pilot_speed_dn.value * (throttle_control-deadband_bottom) / deadband_bottom;
    }else if (throttle_control > deadband_top) {
        // above the deadband
        desired_rate = param->pilot_speed_up.value * (throttle_control-deadband_top) / (1.0f-deadband_top);
    }else{
        // must be in the deadband
        desired_rate = 0.0f;
    }

    return desired_rate;
}

// get_non_takeoff_throttle - a throttle somewhere between min and mid throttle which should not lead to a takeoff
float get_non_takeoff_throttle(void)
{
    return MAX(0,motors->get_throttle_hover()/2.0f);
}

// get_surface_tracking_climb_rate - hold copter at the desired distance above the ground
//      returns climb rate (in cm/s) which should be passed to the position controller
// if use this function, we should set rangefinder_state.alt_healthy=true;
static float target_rangefinder_alt=0.0f;   // desired altitude in cm above the ground
static bool hit_target_rangefinder_alt=false;
static uint32_t hit_time_ms = 0;
void set_target_rangefinder_alt(float alt_target){
	target_rangefinder_alt=alt_target;
	hit_target_rangefinder_alt=false;
}
float get_surface_tracking_climb_rate(float target_rate, float current_alt_target, float dt)
{
	if(!rangefinder_state.alt_healthy||!enable_surface_track){
		  // if don't use rangefinder or rangefinder is not healthy, do not use surface tracking
		  return target_rate;
	}

    static uint32_t last_call_ms = 0;
    float distance_error;
    float velocity_correction;
    float current_alt = get_pos_z();

    uint32_t now = HAL_GetTick();

    // reset target altitude if this controller has just been engaged
    if (now - last_call_ms > RANGEFINDER_TIMEOUT_MS&&robot_sub_mode!=MODE_AUTONAV) {
    	target_rangefinder_alt = rangefinder_state.alt_cm + current_alt_target - current_alt;
    	hit_target_rangefinder_alt=false;
	}
    last_call_ms = now;

    // adjust rangefinder target alt if motors have not hit their limits
    if ((target_rate<0 && !motors->limit.throttle_lower) || (target_rate>0 && !motors->limit.throttle_upper)) {
        target_rangefinder_alt += target_rate * dt;
        hit_target_rangefinder_alt=false;
    }

    /*
      handle rangefinder glitches. When we get a rangefinder reading
      more than RANGEFINDER_GLITCH_ALT_CM different from the current
      rangefinder reading then we consider it a glitch and reject
      until we get RANGEFINDER_GLITCH_NUM_SAMPLES samples in a
      row. When that happens we reset the target altitude to the new
      reading
     */
    float glitch_cm = rangefinder_state.alt_cm - target_rangefinder_alt;
    if(!hit_target_rangefinder_alt&&target_rangefinder_alt>20.0f){
    	if(abs(glitch_cm)<10.0f){
    		hit_target_rangefinder_alt=true;
    		hit_time_ms=now;
    	}
    }
    if(!hit_target_rangefinder_alt||(now-hit_time_ms)<2000){
    	rangefinder_state.glitch_count = 0;
    }else{
    	if (glitch_cm >= RANGEFINDER_GLITCH_ALT_CM) {
			rangefinder_state.glitch_count = MAX(rangefinder_state.glitch_count+1,1);
		} else if (glitch_cm <= -RANGEFINDER_GLITCH_ALT_CM) {
			rangefinder_state.glitch_count = MIN(rangefinder_state.glitch_count-1,-1);
		} else {
			rangefinder_state.glitch_count = 0;
		}
    }
    if (abs(rangefinder_state.glitch_count) >= RANGEFINDER_GLITCH_NUM_SAMPLES) {
        // shift to the new rangefinder reading
        target_rangefinder_alt = rangefinder_state.alt_cm;
        rangefinder_state.glitch_count = 0;
    }
	if(target_rangefinder_alt > param->alt_return.value){//不允许飞机飞到超过巡航高度
		target_rangefinder_alt=param->alt_return.value;
		hit_target_rangefinder_alt=false;
	}
	if(rangefinder_state.alt_cm>param->alt_return.value&&target_rate>0.0f){
		target_rate=0.0f;
	}
    if (rangefinder_state.glitch_count != 0) {
        // we are currently glitching, just use the target rate
        return target_rate;
    }
//    usb_printf_dir("$%d %d;",(uint16_t)target_rangefinder_alt,(uint16_t)rangefinder_state.alt_cm);
    // calc desired velocity correction from target rangefinder alt vs actual rangefinder alt (remove the error already passed to Altitude controller to avoid oscillations)
    distance_error = (target_rangefinder_alt - rangefinder_state.alt_cm) - (current_alt_target - current_alt);
    velocity_correction = distance_error * param->rangefinder_gain.value;
    velocity_correction = constrain_float(velocity_correction, -THR_SURFACE_TRACKING_VELZ_MAX, THR_SURFACE_TRACKING_VELZ_MAX);

    // return combined pilot climb rate + rate to correct rangefinder alt error
    return (target_rate + velocity_correction);
}

float get_rangefinder_alt(void)
{
	return rangefinder_state.alt_cm;
}

float get_rangefinder_alt_target(void)
{
	return target_rangefinder_alt;
}

// set_accel_throttle_I_from_pilot_throttle - smoothes transition from pilot controlled throttle to autopilot throttle
void set_accel_throttle_I_from_pilot_throttle()
{
    // get last throttle input sent to attitude controller
    float pilot_throttle = constrain_float(attitude->get_throttle_in(), 0.0f, 1.0f);
    // shift difference between pilot's throttle and hover throttle into accelerometer I
    pos_control->get_accel_z_pid().set_integrator((pilot_throttle-motors->get_throttle_hover()) * 1000.0f);
}

// get_pilot_desired_angle - transform pilot's roll or pitch input into a desired lean angle
// returns desired angle in degrees
void get_pilot_desired_lean_angles(float &roll_out, float &pitch_out, float angle_max, float angle_limit)
{
    // fetch roll and pitch inputs
    roll_out = get_channel_roll_angle();
    pitch_out = get_channel_pitch_angle();

    // limit max lean angle
    angle_limit = constrain_float(angle_limit, 10.0f, angle_max);

    // scale roll and pitch inputs to ANGLE_MAX parameter range
    float scaler = angle_max/(float)ROLL_PITCH_YAW_INPUT_MAX;
    roll_out *= scaler;
    pitch_out *= scaler;

    // do circular limit
    float total_in = norm(pitch_out, roll_out);
    if (total_in > angle_limit) {
        float ratio = angle_limit / total_in;
        roll_out *= ratio;
        pitch_out *= ratio;
    }

    // do lateral tilt to euler roll conversion
    roll_out = (180/M_PI) * atanf(cosf(pitch_out*(M_PI/180))*tanf(roll_out*(M_PI/180)));

    // roll_out and pitch_out are returned
}

void get_wind_correct_lean_angles(float &roll_d, float &pitch_d, float angle_max)
{
#if USE_WIND
	float wind_pitch_deg=atanf(ekf_wind->wind_x_filt/(GRAVITY_MSS*100.0f))*RAD_TO_DEG;
	float wind_roll_deg=-atanf(ekf_wind->wind_y_filt*cosf(pitch_log)/(GRAVITY_MSS*100.0f))*RAD_TO_DEG;
	roll_d+=constrain_float(wind_roll_deg, -angle_max, angle_max);
	pitch_d+=constrain_float(wind_pitch_deg, -angle_max, angle_max);
#endif
//	usb_printf("pitch:%f|roll:%f\n",pitch_d,roll_d);
}

static uint8_t flow_tick=0;
void get_accel_correct_lean_angles(float &roll_d, float &pitch_d, float angle_max)
{
	if (USE_FLOW&&USE_MAG&&!get_gnss_state()){
		if(opticalflow_state.healthy){
			flow_tick=0;
		}else{
			float acc_pitch_deg=atanf(ax_body/GRAVITY_MSS)*RAD_TO_DEG;
			float acc_roll_deg=-atanf(ay_body*cosf(pitch_log)/GRAVITY_MSS)*RAD_TO_DEG;
			roll_d=constrain_float(acc_roll_deg, -angle_max, angle_max);
			pitch_d=constrain_float(acc_pitch_deg, -angle_max, angle_max);
			flow_tick++;
			if(flow_tick>4){
				pos_control->set_xy_target(get_pos_x(), get_pos_y());
				pos_control->reset_predicted_accel(get_vel_x(), get_vel_y());
				flow_tick=4;
			}
		}
	}
//	usb_printf("pitch:%f|roll:%f\n",pitch_d,roll_d);
}

static bool _return=false;
void set_return(bool set){
	_return=set;
}

bool get_return(void){
	return _return;
}

/******************take off functions start*********************/
static bool _takeoff=false;
static bool _takeoff_running=false;
static float _takeoff_max_speed=0;
static float _takeoff_start_ms=0;
static float _takeoff_alt_delta=0;

void set_takeoff(void){
	if(motors->get_interlock()){
		robot_state_desired=STATE_TAKEOFF;
		_takeoff=true;
	}
}

bool get_takeoff(void){
	if (!ap->land_complete) {
		// can't take off if we're already flying
		return false;
	}
	return _takeoff;
}

bool takeoff_running(void) { return _takeoff_running; }

// start takeoff to specified altitude above home in centimeters
void takeoff_start(float alt_cm)
{
    // calculate climb rate
    const float speed = MAX(param->pilot_speed_up.value*2.0f/3.0f, param->pilot_speed_up.value-50.0f);

    // sanity check speed and target
    if (takeoff_running() || speed <= 0.0f || alt_cm <= 0.0f) {
        return;
    }

    // initialise takeoff state
    _takeoff=false;
    _takeoff_running = true;
    _takeoff_max_speed = speed;
    _takeoff_start_ms = HAL_GetTick();
    _takeoff_alt_delta = alt_cm;
    takeoff_alt=get_pos_z();
    set_return(false);
}

bool takeoff_triggered( float target_climb_rate)
{
    if (!ap->land_complete) {
        // can't take off if we're already flying
        return false;
    }
    if (target_climb_rate <= 0.0f) {
        // can't takeoff unless we want to go up...
        return false;
    }

    return true;
}

void takeoff_stop(void)
{
	_takeoff=false;
	_takeoff_running = false;
	_takeoff_start_ms = 0;
}

// returns pilot and takeoff climb rates
//  pilot_climb_rate is both an input and an output
//  takeoff_climb_rate is only an output
//  has side-effect of turning takeoff off when timeout as expired
void get_takeoff_climb_rates(float& pilot_climb_rate,  float& takeoff_climb_rate)
{
    // return pilot_climb_rate if take-off inactive
    if (!_takeoff_running) {
        takeoff_climb_rate = 0.0f;
        return;
    }

    // acceleration of 100cm/s/s
    static const float TAKEOFF_ACCEL = 100.0f;
    const float takeoff_minspeed = MIN(50.0f, _takeoff_max_speed);
    const float time_elapsed = (HAL_GetTick() - _takeoff_start_ms) * 1.0e-3f;
    const float speed = MIN(time_elapsed * TAKEOFF_ACCEL + takeoff_minspeed, _takeoff_max_speed);

    const float time_to_max_speed = (_takeoff_max_speed - takeoff_minspeed) / TAKEOFF_ACCEL;
    float height_gained;
    if (time_elapsed <= time_to_max_speed) {
        height_gained = 0.5f * TAKEOFF_ACCEL * sq(time_elapsed) + takeoff_minspeed * time_elapsed;
    } else {
        height_gained = 0.5f * TAKEOFF_ACCEL * sq(time_to_max_speed) + takeoff_minspeed * time_to_max_speed +(time_elapsed - time_to_max_speed) * _takeoff_max_speed;
    }

    // check if the takeoff is over
    if (height_gained >= _takeoff_alt_delta) {
    	takeoff_stop();
    }

    // if takeoff climb rate is zero return
    if (speed <= 0.0f) {
        takeoff_climb_rate = 0.0f;
        return;
    }

    // default take-off climb rate to maximum speed
    takeoff_climb_rate = speed;

    // if pilot's commands descent
    if (pilot_climb_rate < 0.0f) {
        // if overall climb rate is still positive, move to take-off climb rate
        if (takeoff_climb_rate + pilot_climb_rate > 0.0f) {
            takeoff_climb_rate = takeoff_climb_rate + pilot_climb_rate;
            pilot_climb_rate = 0.0f;
        } else {
            // if overall is negative, move to pilot climb rate
            pilot_climb_rate = pilot_climb_rate + takeoff_climb_rate;
            takeoff_climb_rate = 0.0f;
        }
    } else { // pilot commands climb
        // pilot climb rate is zero until it surpasses the take-off climb rate
        if (pilot_climb_rate > takeoff_climb_rate) {
            pilot_climb_rate = pilot_climb_rate - takeoff_climb_rate;
        } else {
            pilot_climb_rate = 0.0f;
        }
    }
}
/******************take off functions end*********************/

// set land_complete flag
void set_land_complete(bool b)
{
    ap->land_complete = b;
}

// arm_motors - performs arming process including initialisation of barometer and gyros
//  returns false if arming failed because of pre-arm checks, arming checks or a gyro calibration failure
bool arm_motors(void)
{
	if (!motors->get_interlock()) {
		return false;
	}
	//TODO: add other pre-arm check
	if (!ahrs_healthy||!initial_baro||(PREARM_CHECK&&(use_rangefinder&&!rangefinder_state.enabled)&&(!get_gnss_state()||!get_gnss_stabilize()))||!update_pos||!odom_safe||(USE_ODOMETRY&&odom_2d==0)){
		Buzzer_set_ring_type(BUZZER_ERROR);
		return false;//传感器异常，禁止电机启动
	}
    static bool in_arm_motors = false;

    // exit immediately if already in this function
    if (in_arm_motors) {
        return false;
    }
    in_arm_motors = true;

    // return true if already armed
    if (motors->get_armed()) {
        in_arm_motors = false;
        return true;
    }

    // finally actually arm the motors
    motors->set_armed(true);

    set_soft_armed(true);

    Logger_Enable();

    // flag exiting this function
    in_arm_motors = false;
    takeoff_time=HAL_GetTick();
    Buzzer_set_ring_type(BUZZER_ARMED);

    // return success
    return true;
}

// init_disarm_motors - disarm motors
void disarm_motors(void)
{
    // return immediately if we are already disarmed
    if (!motors->get_armed()) {
        return;
    }

    // we are not in the air
    set_land_complete(true);

    // send disarm command to motors
    motors->set_armed(false);

    set_soft_armed(false);

    //停止日志记录
    Logger_Disable();
    takeoff_time=0;
    Buzzer_set_ring_type(BUZZER_DISARM);
    robot_state_desired=STATE_NONE;//清空状态标志
}

//解锁电机
void unlock_motors(void){
	if (motors->get_interlock()) {
		return;
	}
	//TODO: add other pre-arm check
	if (!ahrs_healthy||!initial_baro||(PREARM_CHECK&&(use_rangefinder&&!rangefinder_state.enabled)&&(!get_gnss_state()||!get_gnss_stabilize()))||!update_pos||!odom_safe||(USE_ODOMETRY&&odom_2d==0)){
		Buzzer_set_ring_type(BUZZER_ERROR);
		return;//传感器异常，禁止电机启动
	}
	// enable output to motors and servos
	set_rcout_enable(true);
	FMU_PWM_Set_Output_Enable();
	motors->set_interlock(true);
	FMU_LED4_Control(false);
	FMU_LED7_Control(true);
	write_gpio2(true);
	send_mavlink_commond_ack((mavlink_channel_t)gcs_channel, MAV_CMD_COMPONENT_ARM_DISARM, MAV_CMD_ACK_OK);
}

//锁定电机
void lock_motors(void){
	if (!motors->get_interlock()) {
		return;
	}
	// disable output to motors and servos
	set_rcout_enable(false);
	FMU_PWM_Set_Output_Disable();
	motors->set_interlock(false);
	FMU_LED4_Control(true);
	FMU_LED7_Control(false);
	write_gpio2(false);
	send_mavlink_commond_ack((mavlink_channel_t)gcs_channel, MAV_CMD_COMPONENT_ARM_DISARM, MAV_CMD_ACK_ERR_FAIL);
}

// counter to verify landings
static uint32_t land_detector_count = 0;
// update_land_detector - checks if we have landed and updates the ap.land_complete flag
// called at 100hz
static void update_land_detector(void)
{
	ahrs->check_vibration();
	if(rangefinder_state.enabled&&(HAL_GetTick()-rangefinder_state.last_update_ms)>300){
		rangefinder_state.enabled=false;
		rangefinder_state.alt_healthy=false;
	}
	//******************落地前********************
	if((get_vel_z()<0)&&(ekf_baro->vel_2d<100)&&(pos_control->get_desired_velocity().z<0)&&(get_vib_value()>param->vib_land.value)&&(motors->get_throttle()<motors->get_throttle_hover())&&(!motors->limit.throttle_lower)){//TODO:降落时防止弹起来
		disarm_motors();
	}
	//******************落地后ls*********************

    // land detector can not use the following sensors because they are unreliable during landing
    // barometer altitude :                 ground effect can cause errors larger than 4m
    // EKF vertical velocity or altitude :  poor barometer and large acceleration from ground impact
    // earth frame angle or angle error :   landing on an uneven surface will force the airframe to match the ground angle
    // gyro output :                        on uneven surface the airframe may rock back an forth after landing
    // range finder :                       tend to be problematic at very short distances
    // input throttle :                     in slow land the input throttle may be only slightly less than hover

    if (!motors->get_armed()) {
        // if disarmed, always landed.
        set_land_complete(true);
        // we've landed so reset land_detector
        land_detector_count = 0;
    } else if (ap->land_complete) {
        // if throttle output is high then clear landing flag
        if (motors->get_throttle() > get_non_takeoff_throttle()) {
            set_land_complete(false);
        }
        // we've landed so reset land_detector
        land_detector_count = 0;
    } else {

        // check that the average throttle output is near minimum (less than 12.5% hover throttle)
        bool motor_at_lower_limit = motors->limit.throttle_lower && attitude->is_throttle_mix_min();

        // check that vertical speed is within 1m/s of zero
        bool descent_rate_low = fabsf(get_vel_z()) < 100;

        // if we have a healthy rangefinder only allow landing detection below 2 meters
        bool rangefinder_check = (!rangefinder_state.alt_healthy || rangefinder_state.alt_cm < LAND_RANGEFINDER_MIN_ALT_CM);

        if (motor_at_lower_limit && descent_rate_low && rangefinder_check) {
            // landed criteria met - increment the counter and check if we've triggered
            if( land_detector_count < ((float)LAND_DETECTOR_TRIGGER_SEC)/0.01) {
                land_detector_count++;
            } else {
            	disarm_motors();
            }
        } else {
            // we've sensed movement up or down so reset land_detector
            land_detector_count = 0;
        }
    }
}

// update_throttle_thr_mix - sets motors throttle_low_comp value depending upon vehicle state
//  low values favour pilot/autopilot throttle over attitude control, high values favour attitude control over throttle
//  has no effect when throttle is above hover throttle
static void update_throttle_thr_mix()
{
    // if disarmed or landed prioritise throttle
    if(!motors->get_armed() || ap->land_complete) {
        attitude->set_throttle_mix_min();
        return;
    }

    if (has_manual_throttle()) {
        // manual throttle
        if(get_channel_throttle() <= 0) {
            attitude->set_throttle_mix_min();
        } else {
            attitude->set_throttle_mix_man();
        }
    } else {
        // autopilot controlled throttle

        // check for aggressive flight requests - requested roll or pitch angle below 15 degrees
        const Vector3f angle_target = attitude->get_att_target_euler_d();
        bool large_angle_request = (norm(angle_target.x, angle_target.y) > LAND_CHECK_LARGE_ANGLE_DEG);

        // check for large external disturbance - angle error over 30 degrees
        const float angle_error = attitude->get_att_error_angle_deg();
        bool large_angle_error = (angle_error > LAND_CHECK_ANGLE_ERROR_DEG);

        // check for large acceleration - falling or high turbulence
        Vector3f accel_ef = get_accel_ef();
        accel_ef.z += GRAVITY_MSS;
        bool accel_moving = (accel_ef.length() > LAND_CHECK_ACCEL_MOVING);

        // check for requested decent
        bool descent_not_demanded = pos_control->get_desired_velocity().z >= 0.0f;

        if ( large_angle_request || large_angle_error || accel_moving || descent_not_demanded) {
            attitude->set_throttle_mix_max();
        } else {
            attitude->set_throttle_mix_min();
        }
    }
}

// throttle_loop - should be run at 100 hz
void throttle_loop(void){
	float ch5=get_channel_5();
	if(ch5>=0.7&&ch5<=1.0){
		use_rangefinder=false;
		set_gnss_state(false);
	}else{
		use_rangefinder=true;
	}
	// update estimated throttle required to hover
	update_throttle_hover();
    // update throttle_low_comp value (controls priority of throttle vs attitude control)
	update_throttle_thr_mix();
	//checks if we have landed and updates the ap.land_complete flag
	update_land_detector();
}

//手势开关电机；
//油门0，偏航最大时启动电机；
//油门0，偏航最小时关闭电机。
//注意不是手动油门时，只有在降落状态时才能用手势关闭电机。
void arm_motors_check(void){
	if(!rc_channels_healthy()){
		return;
	}
	static int16_t arming_counter;
	float throttle=get_channel_throttle();
	// ensure throttle is down
	if (throttle > 0.05) {
		arming_counter = 0;
		return;
	}

	float tmp = get_channel_yaw();

	// full right
	if (tmp > 0.9) {

		// increase the arming counter to a maximum of 1 beyond the auto trim counter
		if( arming_counter < 250 ) {
			arming_counter++;
		}

		// arm the motors and configure for flight
		if (arming_counter == 250 && !motors->get_armed()) {
			if(!arm_motors()){
				arming_counter=0;
			}
		}

	// full left
	}else if (tmp < -0.9) {
		if ((!has_manual_throttle() && !ap->land_complete) || (robot_main_mode!=MODE_AIR)) {
			arming_counter = 0;
			return;
		}

		// increase the counter to a maximum of 1 beyond the disarm delay
		if( arming_counter < 250 ) {
			arming_counter++;
		}

		// disarm the motors
		if (arming_counter == 250 && motors->get_armed()) {
			disarm_motors();
		}

	// Yaw is centered so reset arming counter
	}else{
		arming_counter = 0;
	}
}

#define THROTTLE_ZERO_DEBOUNCE_TIME_MS 400
// set_throttle_zero_flag - set throttle_zero flag from debounced throttle control
// throttle_zero is used to determine if the pilot intends to shut down the motors
// Basically, this signals when we are not flying.  We are either on the ground
// or the pilot has shut down the copter in the air and it is free-falling
void set_throttle_zero_flag(float throttle_control)
{
    static uint32_t last_nonzero_throttle_ms = 0;
    uint32_t tnow_ms = HAL_GetTick();

    // if not using throttle interlock and non-zero throttle and not E-stopped,
    // or using motor interlock and it's enabled, then motors are running,
    // and we are flying. Immediately set as non-zero
    if (throttle_control > 0.01 && get_soft_armed()) {
        last_nonzero_throttle_ms = tnow_ms;
        ap->throttle_zero = false;
    } else if (tnow_ms - last_nonzero_throttle_ms <= THROTTLE_ZERO_DEBOUNCE_TIME_MS) {
        ap->throttle_zero = false;
    } else {
    	ap->throttle_zero = true;
    }
}

//channel8 < 0.1时解锁电机, channel8 > 0.9时锁定电机
//channel6 短按解锁,长按锁定
static uint8_t disarm_counter=0;
void lock_motors_check(void){
	if(!rc_channels_healthy()){
		return;
	}
	set_throttle_zero_flag(get_channel_throttle());
#if USE_CH8_LOCK
	float ch8=get_channel_8();
	if(ch8>0&&ch8<0.1){
		disarm_counter=0;
		unlock_motors();
	}else if(ch8>0.9&&disarm_counter<=10){
		if(disarm_counter==10){
			disarm_motors();
			lock_motors();
		}else{
			disarm_counter++;
		}
	}else{
		return;
	}
#else
	float ch6=get_channel_6();
	if(ch6>0&&ch6<0.1){
		if (!motors->get_interlock()&&disarm_counter<50&&disarm_counter>0){
			unlock_motors();
		}
		disarm_counter=0;
	}else if(ch6>0.9){
		disarm_counter++;
		if(disarm_counter>200){
			disarm_counter=200;
		}
		if(motors->get_interlock()&&disarm_counter==200){
			disarm_motors();
			lock_motors();
		}
	}
#endif
}

void zero_throttle_and_relax_ac(void)
{
    motors->set_desired_spool_state(Motors::DESIRED_SPIN_WHEN_ARMED);
    // multicopters do not stabilize roll/pitch/yaw when disarmed
    attitude->set_throttle_out_unstabilized(0.0f, true, param->throttle_filt.value);
}

/********回调函数：在SD卡中写入日志数据名称******
 * ***********************************
 * 注意sd_log_write()函数最多可以一次记录128个字符的字符串
 * 每个名称后面需要加入空格字符，像这样 "%8s "
 * ***********************************
 * ***********************************/
void Logger_Cat_Callback(void){
	sd_log_write("%8s %8s %8s %8s %8s %8s %8s ",//LOG_SENSOR
			"t_ms", "accx", "accy", "accz", "gyrox", "gyroy", "gyroz");
	osDelay(2);
	sd_log_write("%8s %8s %8s %8s %8s %8s %8s %8s %8s ",//LOG_SENSOR
			"magx", "magy", "magz", "baro", "voltage", "current", "sat_num", "wind_x", "wind_y");
	osDelay(2);
	sd_log_write("%8s %8s %8s %8s %8s %8s %8s %8s %8s %8s ",//LOG_EULER
			"pitch_log", "roll_log", "yaw_log", "pitchd", "rolld", "yawd", "rtk_yawd", "lat_noi", "lon_noi", "alt_noi");
	osDelay(2);
	sd_log_write("%8s %8s %8s %8s %8s %8s %8s %8s ",//LOG_ACCEL_EARTH_FRAME and VIB
			"gyrox_t", "gyroy_t", "gyroz_t", "efx", "efy", "efz", "vib_vl", "vib_ag");
	osDelay(2);
	sd_log_write("%8s %8s %8s %8s %8s %8s %8s %8s %8s ",//LOG_POS_Z
			"barofilt", "alt_t", "pos_z", "vel_z_t", "vel_z", "rf_alt", "rf_alt_t", "rtk_alt", "rtk_velz");
	osDelay(2);
	sd_log_write("%8s %8s %8s %8s %8s %8s %8s %8s %8s %8s %8s ",//LOG_POS_XY
			"vt_z","flow_vx", "ned_x", "ned_vx", "pos_x", "vel_x", "flow_vy", "ned_y", "ned_vy", "pos_y", "vel_y");
	osDelay(2);
	sd_log_write("%8s %8s %8s %8s %8s %8s %8s %8s %8s ",//LOG_VEL_PID_XYZ
			"v_p_x", "v_i_x", "v_d_x", "v_p_y", "v_i_y", "v_d_y", "a_p_z", "a_i_z", "a_d_z");
	osDelay(2);
	sd_log_write("%8s %8s %8s %8s %8s %8s %8s %8s ",//LOG_RCIN
			"roll", "pitch", "yaw", "thr", "ch5", "ch6", "ch7", "ch8");
	osDelay(2);
	sd_log_write("%8s %8s %8s %8s %8s %8s %8s %8s %8s %8s %8s ",//OFFBOARD
			"uwb_x", "uwb_y", "odom_x", "odom_y", "odom_z", "mav_x_t", "mav_y_t", "mav_z_t", "goal_x", "goal_y", "goal_z");
	osDelay(2);
	sd_log_write("%8s %8s %8s %8s %8s %8s %8s %8s ",//OFFBOARD
			"mav_v_x", "mav_v_y", "mav_v_z", "mav_a_x", "mav_a_y", "mav_a_z", "mav_yaw", "mav_rate");
	osDelay(2);
	sd_log_write("%8s %8s %8s %8s %8s %8s ",//LOG_ANCHOR
			"dis1", "dis2", "dis3", "dis4", "lat", "lon");
	osDelay(2);
	sd_log_write("%8s %8s %8s %8s %8s %8s %8s %8s ",//LOG_RCOUT
			"motor1", "motor2", "motor3", "motor4", "motor5", "motor6", "motor7", "motor8");
	osDelay(2);
	sd_log_write("%8s %8s %8s %8s %8s %8s %8s ",//LOG_TARGET
			"pos_x_t", "pos_y_t", "vel_x_t", "vel_y_t", "acc_x_t", "acc_y_t", "acc_z_t");
	osDelay(2);
	sd_log_write("%8s %8s %8s %8s %8s %8s %8s %8s %8s ",//LOG_RATE_CONTROL
			"roll_p", "roll_i", "roll_d", "pitch_p", "pitch_i", "pitch_d", "yaw_p", "yaw_i", "yaw_d");
	osDelay(2);
	sd_log_write("%8s %8s %8s %8s %8s %8s %8s %8s ",//LOG_MOTOR_CONTROL
			"roll_t", "pitch_t", "yaw_t", "p_out", "r_out", "y_out", "t_out", "t_hover");
	osDelay(2);
	//add other loggers
}

/********回调函数：在SD卡中写入日志数据数值********
 * *************************************
 * 注意sd_log_write()函数最多可以一次记录128个字符的字符串
 * 每个数据后面需要加入空格字符，像这样 "%8.3f "
 * ***********************************
 * ***********************************/
void Logger_Data_Callback(void){
	sd_log_write("%8ld %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f ",//LOG_SENSOR
			HAL_GetTick(), get_accel_filt().x, get_accel_filt().y, get_accel_filt().z, get_gyro_filt().x, get_gyro_filt().y, get_gyro_filt().z);
	osDelay(2);
	sd_log_write("%8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8d %8.3f %8.3f ",//LOG_SENSOR
			get_mag_filt().x, get_mag_filt().y, get_mag_filt().z, spl06_data.baro_alt, get_batt_volt(), get_batt_current(), gps_position->satellites_used, ekf_wind->wind_x_filt, ekf_wind->wind_y_filt);
	osDelay(2);
	sd_log_write("%8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f ",//LOG_EULER
			pitch_log*RAD_TO_DEG, roll_log*RAD_TO_DEG, yaw_log*RAD_TO_DEG, ahrs_pitch_deg(), ahrs_roll_deg(), ahrs_yaw_deg(), gps_position->heading, gps_position->lat_noise, gps_position->lon_noise, gps_position->alt_noise);
	osDelay(2);
	sd_log_write("%8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f ",//LOG_ACCEL_EARTH_FRAME and VIB
			attitude->rate_bf_targets().x, attitude->rate_bf_targets().y, attitude->rate_bf_targets().z, get_accel_ef().x, get_accel_ef().y, get_accel_ef().z, get_vib_value(), get_vib_angle_z());
	osDelay(2);
	sd_log_write("%8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f ",//LOG_POS_Z
			get_baroalt_filt(), pos_control->get_pos_target().z, get_pos_z(), pos_control->get_vel_target_z(), get_vel_z(), get_rangefinder_alt(), get_rangefinder_alt_target(), -get_ned_pos_z(), -get_ned_vel_z());
	osDelay(2);
	sd_log_write("%8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f ",//LOG_POS_XY
			ekf_baro->get_vt(), opticalflow_state.vel.x, get_ned_pos_x(), get_ned_vel_x(), get_pos_x(), get_vel_x(), opticalflow_state.vel.y, get_ned_pos_y(), get_ned_vel_y(), get_pos_y(), get_vel_y());
	osDelay(2);
	sd_log_write("%8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f ",//LOG_VEL_PID_XYZ
			pos_control->get_vel_xy_pid().get_p().x, pos_control->get_vel_xy_pid().get_integrator().x, pos_control->get_vel_xy_pid().get_d().x,
			pos_control->get_vel_xy_pid().get_p().y, pos_control->get_vel_xy_pid().get_integrator().y, pos_control->get_vel_xy_pid().get_d().y,
			pos_control->get_accel_z_pid().get_p(), pos_control->get_accel_z_pid().get_integrator(), pos_control->get_accel_z_pid().get_d());
	osDelay(2);
	sd_log_write("%8d %8d %8d %8d %8d %8d %8d %8d ",//LOG_RCIN
			input_channel_roll(), input_channel_pitch(), input_channel_yaw(), input_channel_throttle(), input_channel_5(), input_channel_6(), input_channel_7(), input_channel_8());
	osDelay(2);
	sd_log_write("%8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f ",//OFFBOARD
			get_uwb_x(), get_uwb_y(), get_odom_x(), get_odom_y(), get_odom_z(), get_mav_x_target(), get_mav_y_target(), get_mav_z_target(), set_goal_point.x, set_goal_point.y, set_goal_point.z);
	osDelay(2);
	sd_log_write("%8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f ",//OFFBOARD
			get_mav_vx_target(), get_mav_vy_target(), get_mav_vz_target(), get_mav_ax_target(), get_mav_ay_target(), get_mav_az_target(), get_mav_yaw_target(), get_mav_yaw_rate_target());
	osDelay(2);
	sd_log_write("%8d %8d %8d %8d %8ld %8ld ",//LOG_ANCHOR
			uwb->Anchordistance[0], uwb->Anchordistance[1], uwb->Anchordistance[2], uwb->Anchordistance[3], gps_position->lat, gps_position->lon);
	osDelay(2);
	sd_log_write("%8d %8d %8d %8d %8d %8d %8d %8d ",//LOG_RCOUT
			pwm_channel.motor[0], pwm_channel.motor[1], pwm_channel.motor[2], pwm_channel.motor[3], pwm_channel.motor[4], pwm_channel.motor[5], pwm_channel.motor[6], pwm_channel.motor[7]);
	osDelay(2);
	sd_log_write("%8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f ",//LOG_TARGET
			pos_control->get_pos_target().x, pos_control->get_pos_target().y, pos_control->get_vel_target().x, pos_control->get_vel_target().y,	pos_control->get_accel_target().x, pos_control->get_accel_target().y, pos_control->get_accel_target().z);
	osDelay(2);
	sd_log_write("%8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f ",//LOG_RATE_CONTROL
			attitude->get_rate_roll_pid().get_p(), attitude->get_rate_roll_pid().get_integrator(), attitude->get_rate_roll_pid().get_d(), attitude->get_rate_pitch_pid().get_p(), attitude->get_rate_pitch_pid().get_integrator(), attitude->get_rate_pitch_pid().get_d(),
			attitude->get_rate_yaw_pid().get_p(), attitude->get_rate_yaw_pid().get_integrator(), attitude->get_rate_yaw_pid().get_d());
	osDelay(2);
	sd_log_write("%8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f ",//LOG_MOTOR_CONTROL
			attitude->get_att_target_euler_d().x, attitude->get_att_target_euler_d().y, attitude->get_att_target_euler_d().z,
			motors->get_pitch(), motors->get_roll(), motors->get_yaw(), motors->get_throttle(), motors->get_throttle_hover());
	osDelay(2);
	//add other loggers
}

void Log_To_File(Log_Type log_type)
{
	switch(log_type){
		case LOG_CAT:
			Logger_Cat_Callback();
			break;
		case LOG_DATA:
			Logger_Data_Callback();
			break;
		case LOG_END:
			sd_log_end();
			break;
		default:
			break;
	}
}

void Logger_Enable(void)
{
  m_Logger_Status=Logger_Open;
}

void Logger_Disable(void)
{
	if((m_Logger_Status == Logger_Record)||(m_Logger_Status == Logger_Open)){
	  m_Logger_Status=Logger_Close;
	}
}

void Logger_Write_Gnss(void)
{
  m_Logger_Status=Logger_Gnss_Write;
}

void Logger_Read_Gnss(void)
{
  m_Logger_Status=Logger_Gnss_Read;
}



void Logger_Update(void){
	switch (m_Logger_Status){
		case Logger_Idle:
			osDelay(100);
			break;
		case Logger_Close:
			sd_log_close();
			m_Logger_Status = Logger_Idle;
			osDelay(10);
			break;
		case Logger_Open:
			if(sd_log_start()!=FR_OK){//没插卡
				m_Logger_Status=Logger_Idle;
				break;
			}
			Log_To_File(LOG_CAT);
			Log_To_File(LOG_END);
			m_Logger_Status = Logger_Record;
			break;
		case Logger_Record:
			Log_To_File(LOG_DATA);
			Log_To_File(LOG_END);
			break;
		case Logger_Gnss_Write:
			Write_Gnss_File();
			m_Logger_Status = Logger_Idle;
			break;
		case Logger_Gnss_Read:
			Read_Gnss_File();
			m_Logger_Status = Logger_Idle;
			break;
		default:
			m_Logger_Status = Logger_Idle;
			break;
   }
}

static mavlink_message_t msg_scaled_imu, msg_attitude_quaternion;
static mavlink_scaled_imu_t scaled_imu;
static mavlink_attitude_quaternion_t attitude_quaternion;
void comm_send_callback(void){
	if(offboard_connected){
		scaled_imu.time_boot_ms=HAL_GetTick();
		scaled_imu.xacc=(int16_t)(get_accel_filt().x*1000);
		scaled_imu.yacc=(int16_t)(get_accel_filt().y*1000);
		scaled_imu.zacc=(int16_t)(get_accel_filt().z*1000);
		scaled_imu.xgyro=(int16_t)(get_gyro_filt().x*1000);
		scaled_imu.ygyro=(int16_t)(get_gyro_filt().y*1000);
		scaled_imu.zgyro=(int16_t)(get_gyro_filt().z*1000);
		mavlink_msg_scaled_imu_encode(mavlink_system.sysid, mavlink_system.compid, &msg_scaled_imu, &scaled_imu);
		mavlink_send_buffer((mavlink_channel_t)offboard_channel, &msg_scaled_imu);

		attitude_quaternion.time_boot_ms=HAL_GetTick();
		attitude_quaternion.q1=ahrs->quaternion2.q1;
		attitude_quaternion.q2=ahrs->quaternion2.q2;
		attitude_quaternion.q3=ahrs->quaternion2.q3;
		attitude_quaternion.q4=ahrs->quaternion2.q4;
		attitude_quaternion.rollspeed=get_gyro_filt().x;
		attitude_quaternion.pitchspeed=get_gyro_filt().y;
		attitude_quaternion.yawspeed=get_gyro_filt().z;
		mavlink_msg_attitude_quaternion_encode(mavlink_system.sysid, mavlink_system.compid, &msg_attitude_quaternion, &attitude_quaternion);
		mavlink_send_buffer((mavlink_channel_t)offboard_channel, &msg_attitude_quaternion);
	}
	flush_serial_data((mavlink_channel_t)offboard_channel);
	flush_usb_data();
}

void uwb_send_data(void){
#if COMM_UWB==MAV_COMM
	uint32_t time=HAL_GetTick();
	if((time-time_last_heartbeat[MAVLINK_COMM_5])>5000&&(HeartBeatFlags&(EVENTBIT_HEARTBEAT_COMM_0<<MAVLINK_COMM_5))){
		HeartBeatFlags&=(0xFF^(EVENTBIT_HEARTBEAT_COMM_0<<MAVLINK_COMM_5));
		return;
	}

	//姿态+位置
	global_attitude_position.pitch=ahrs_pitch_rad();
	global_attitude_position.roll=ahrs_roll_rad();
	global_attitude_position.yaw=ahrs_yaw_rad();
	if(use_uwb){
		global_attitude_position.x=get_pos_x()*cosf(uwb_yaw_delta)-get_pos_y()*sinf(uwb_yaw_delta);
		global_attitude_position.y=get_pos_x()*sinf(uwb_yaw_delta)+get_pos_y()*cosf(uwb_yaw_delta);
	}else{
		global_attitude_position.x=get_pos_x();
		global_attitude_position.y=get_pos_y();
	}
	global_attitude_position.z=get_pos_z();
	global_attitude_position.usec=time;
	mavlink_msg_global_vision_position_estimate_encode(mavlink_system.sysid, mavlink_system.compid, &msg_global_attitude_position, &global_attitude_position);
	uwb_send_mavlink_buffer(&msg_global_attitude_position);

	//经纬高+速度
	if(use_uwb){
		global_position_int.lat=(int32_t)(uwb_pos.x*cosf(uwb_yaw_delta)-uwb_pos.y*sinf(uwb_yaw_delta));//cm
		global_position_int.lon=(int32_t)(uwb_pos.x*sinf(uwb_yaw_delta)+uwb_pos.y*cosf(uwb_yaw_delta));//cm
		global_position_int.alt=(int32_t)uwb_pos.z;//cm
	}else{
		global_position_int.lat=gps_position->lat;//deg*1e7
		global_position_int.lon=gps_position->lon;//deg*1e7
		global_position_int.alt=gps_position->alt;//mm
	}
	global_position_int.relative_alt=(int32_t)(rangefinder_state.alt_cm*10);//对地高度 mm
	global_position_int.hdg=(uint16_t)gps_position->satellites_used|((uint16_t)gps_position->heading_status<<8)|((uint16_t)gps_position->fix_type<<12);//卫星数+定向状态+定位状态
	global_position_int.vx=get_vel_x(); //速度cm/s
	global_position_int.vy=get_vel_y(); //速度cm/s
	global_position_int.vz=get_vel_z(); //速度cm/s
	if(takeoff_time>0){
		global_position_int.time_boot_ms=time-takeoff_time;//起飞时间 ms
	}else{
		global_position_int.time_boot_ms=0;
	}
	mavlink_msg_global_position_int_encode(mavlink_system.sysid, mavlink_system.compid, &msg_global_position_int, &global_position_int);
	uwb_send_mavlink_buffer(&msg_global_position_int);
#endif
}

void motors_test_update(void){
	if(motor_test_type>0.5f&&(HAL_GetTick()-motor_test_start_time)<(uint32_t)(motor_test_timeout*1000.0f)){
		motors->set_throttle_passthrough_for_motor((uint8_t)motor_test_num, motor_test_throttle);
	}
}

/*****************************************************************
 * *******************code for test and debug*********************
 *****************************************************************/
void debug(void){
//	usb_printf("pos_x:%f,pos_y:%f\n",get_odom_x(),get_odom_y());
//	usb_printf("l:%f\n",get_dcm_matrix().c.z);
//	usb_printf("nedx:%f|nedy:%f|x:%f|y:%f|vx:%f|vy:%f\n", get_ned_pos_x(), get_ned_pos_y(), get_pos_x(), get_pos_y(),get_vel_x(), get_vel_y());
//	usb_printf("gps_position lat:%lf ,lon:%lf ,alt:%lf \r\n" , (double)gps_position->lat/10000000.0,(double)gps_position->lon/10000000.0,(double)gps_position->alt/1000000.0);
//	usb_printf("l:%d|%d|%d\n",*(__IO uint8_t*)((uint32_t)0x081D0000),*(__IO uint8_t*)((uint32_t)0x081D0001),*(__IO uint8_t*)((uint32_t)0x081D0002));
//	usb_printf("l:%d\n",dataflash->get_addr_num_max());
//	dataflash->get_param_float(param->acro_y_expo.num, param->acro_y_expo.value);
//	usb_printf("p:%f,%f,%f\n",uwb->uwb_position.x, uwb->uwb_position.y, uwb->uwb_position.z);
//	dataflash->set_param_float(param->acro_yaw_p.num, 3.6);
//	dataflash->get_param_float(param->acro_yaw_p.num, param->acro_yaw_p.value);
//	usb_printf("%f\n",param->acro_yaw_p.value);
//	usb_printf("%f\n",motors->get_throttle());
//	usb_printf("%f|%f\n",param.throttle_filt.value,param.angle_max.value);
//	usb_printf("x:%f ",param->accel_offsets.value.x);
//	usb_printf("lat:%d \n",gps_position->fix_type);
//	usb_printf("y:%f ",param->accel_offsets.value.y);
//	usb_printf("z:%f\n",param->accel_offsets.value.z);
//	usb_printf("baro:%f\n",spl06_data.baro_alt);
//	usb_printf("temp:%f\n",spl06_data.temp);
//	usb_printf("alt:%f\n",get_rangefinder_alt());
//	float cos_tilt = ahrs_cos_pitch() * ahrs_cos_roll();
//	usb_printf("%f|%f|%f|%f\n", pitch_deg, roll_deg, yaw_deg, accel_ef.z);
//	usb_printf("l:%f\n",get_mag_filt().length());
//	usb_printf("v:%f,i:%f\n",get_batt_volt(),get_batt_current());
//	usb_printf("gx:%f|gy:%f|gz:%f\n", gyro_filt.x, gyro_filt.y, gyro_filt.z);
//	usb_printf("mx:%f|my:%f|mz:%f\n", mag.x, mag.y, mag.z);
//	usb_printf("x:%f,y:%f,z:%f\n",gyro_offset.x,gyro_offset.y,gyro_offset.z);
//	usb_printf("accelx:%f,accely:%f,accelz:%f\n",accel_correct.x,accel_correct.y,accel_correct.z);
//	usb_printf("accelx:%f,accely:%f,accelz:%f\n ",accel_filt.x,accel_filt.y,accel_filt.z);
//	usb_printf("accelx:%f,accely:%f,accelz:%f\n ",accel_ef.x,accel_ef.y,accel_ef.z);
//	usb_printf("x:%f,y:%f,z:%f\n",gyro_offset.x,gyro_offset.y,gyro_offset.z);
//	usb_printf("ax:%f, ay:%f, az:%f, accelx:%f,accely:%f,accelz:%f\n ",dcm_matrix.a.x, dcm_matrix.a.y, dcm_matrix.a.z, accel_filt.x,accel_filt.y,accel_filt.z);
//	usb_printf("ax:%f,ay:%f,az:%f\n",mpu9250_data.accf.x,mpu9250_data.accf.y,mpu9250_data.accf.z);
//	usb_printf("%f,%f,%f\n",mpu9250_data.gyrof.x, mpu9250_data.gyrof.y, mpu9250_data.gyrof.z);
//	usb_printf("%f,%f,%f\n",attitude->get_att_target_euler_d().x, attitude->get_att_target_euler_d().y, attitude->get_att_target_euler_d().z);
//	usb_printf("hover:%f|%f\n",param->t_hover_update_min.value,param->t_hover_update_max.value);
//	usb_printf("m:%d|%d\n",param.motor_type.value, param.robot_type.value);
//	usb_printf("pos_x:%f|%f,pos_y:%f|%f\n",get_pos_x(),get_vel_x(),get_pos_y(),get_vel_y());
//	usb_printf("pitch:%f|roll:%f|yaw:%f\n", pitch_rad, roll_rad, yaw_rad);
//	usb_printf("vib:%f\n", param->vib_land.value);
//	s2_printf("x:%f,y:%f\n", x_target, y_target);
//	usb_printf("tar:%f\n", target_rangefinder_alt);
//	usb_printf("pos_z:%f|%f|%f|%f\n",get_baroalt_filt(),get_pos_z(),get_vel_z(),accel_ef.z);
//	usb_printf("speed:%f\n",param->auto_land_speed.value);
//	usb_printf("z:%f\n",attitude->get_angle_roll_p().kP());
//	usb_printf("r:%f,p:%f,y:%f,t:%f,5:%f,6:%f,7:%f,8:%f\n",get_channel_roll(),get_channel_pitch(),get_channel_yaw(), get_channel_throttle(),get_channel_5(),get_channel_6(),get_channel_7(),get_channel_8());
//	usb_printf("0:%f,1:%f,4:%f,5:%f\n",motors->get_thrust_rpyt_out(0),motors->get_thrust_rpyt_out(1),motors->get_thrust_rpyt_out(4), motors->get_thrust_rpyt_out(5));
//	usb_printf("roll:%f,pitch:%f,yaw:%f,throttle:%f\n",motors->get_roll(),motors->get_pitch(),motors->get_yaw(), motors->get_throttle());
//	usb_printf("yaw:%f,yaw_throttle:%f\n",yaw_deg,motors->get_yaw());
//	usb_printf("c:%f,p:%f\n",compass_calibrate(),param.mag_offsets.value.x);
//	usb_printf("i:%f|%f\n",param->vel_pid_integrator.value.x, param->vel_pid_integrator.value.y);
//	usb_printf("i:%f|%f\n",attitude->get_rate_roll_pid().get_integrator(), attitude->get_rate_pitch_pid().get_integrator());
//	usb_printf("ox:%f, oy:%f, oz:%f\n",param->mag_offsets.value.x, param->mag_offsets.value.y, param->mag_offsets.value.z);
//	usb_printf("dx:%f, dy:%f, dz:%f\n",param->mag_diagonals.value.x, param->mag_diagonals.value.y, param->mag_diagonals.value.z);
//	usb_printf("odx:%f, ody:%f, odz:%f\n",param->mag_offdiagonals.value.x, param->mag_offdiagonals.value.y, param->mag_offdiagonals.value.z);
//	usb_printf("motor:%d|%d|%d|%d\n",pwm_channel.motor[0], motors->get_armed(), get_soft_armed(), motors->get_interlock());
//	usb_printf("declination:%f\n",Declination::get_declination(40.152126, 116.317121));
//  usb_printf("vib_value:%f, vib_angle:%f\n", get_vib_value(), get_vib_angle_z());
//	usb_printf("point:%d,%f,%f,%f\n",sdlog->gnss_point_num,sdlog->gnss_point[0].x,sdlog->gnss_point[0].y,sdlog->gnss_point[0].z);
//	Servo_Set_Value(2,1500);
//	Servo_Set_Value(3,1500);
//	usb_printf("target:%f|%f|%f|%f\n",get_mav_x_target(), get_mav_y_target(), get_mav_z_target(), get_mav_yaw_target());
}
