/*
 * common.h
 *
 *  Created on: 2020.07.16
 *      Author: JackyPan
 *
 *  @brief: this file should be included by all the cpp files
 */
#pragma once
/* Define to prevent recursive inclusion -------------------------------------*/

#include "hal.h"
#include "math/math_inc.h"
#include "filter/LowPassFilter.h"
#include "filter/LowPassFilter2p.h"
#include "filter/DerivativeFilter.h"

/* *************************************************
 * ****************Dev code begin*******************/
// Warning! Developer can add your new code here!

/* ****************Dev code end*********************
 * *************************************************/

////////////////////////////////////////////////////
// Attitude and Position Control pid parameters ////
////////////////////////////////////////////////////
#define AC_ATTITUDE_CONTROL_ANGLE_ROLL_P                      4.5f
#define AC_ATTITUDE_CONTROL_ANGLE_PITCH_P                     4.5f
#define AC_ATTITUDE_CONTROL_ANGLE_YAW_P                       4.5f

// default rate controller PID gains
#ifndef AC_ATC_MULTI_RATE_PITCH_P
  # define AC_ATC_MULTI_RATE_PITCH_P          0.05f
#endif

#ifndef AC_ATC_MULTI_RATE_PITCH_I
  # define AC_ATC_MULTI_RATE_PITCH_I          0.15f
#endif

#ifndef AC_ATC_MULTI_RATE_PITCH_D
  # define AC_ATC_MULTI_RATE_PITCH_D          0.0015f
#endif

#ifndef AC_ATC_MULTI_RATE_ROLL_P
  # define AC_ATC_MULTI_RATE_ROLL_P           0.05f
#endif

#ifndef AC_ATC_MULTI_RATE_ROLL_I
  # define AC_ATC_MULTI_RATE_ROLL_I           0.15f
#endif

#ifndef AC_ATC_MULTI_RATE_ROLL_D
  # define AC_ATC_MULTI_RATE_ROLL_D           0.0015f
#endif

#ifndef AC_ATC_MULTI_RATE_RP_IMAX
 # define AC_ATC_MULTI_RATE_RP_IMAX         0.3f
#endif
#ifndef AC_ATC_MULTI_RATE_RP_FILT_HZ
 # define AC_ATC_MULTI_RATE_RP_FILT_HZ      20.0f
#endif
#ifndef AC_ATC_MULTI_RATE_YAW_P
 # define AC_ATC_MULTI_RATE_YAW_P           0.18f
#endif
#ifndef AC_ATC_MULTI_RATE_YAW_I
 # define AC_ATC_MULTI_RATE_YAW_I           0.018f
#endif
#ifndef AC_ATC_MULTI_RATE_YAW_D
 # define AC_ATC_MULTI_RATE_YAW_D           0.0f
#endif
#ifndef AC_ATC_MULTI_RATE_YAW_IMAX
 # define AC_ATC_MULTI_RATE_YAW_IMAX        0.3f
#endif
#ifndef AC_ATC_MULTI_RATE_YAW_FILT_HZ
 # define AC_ATC_MULTI_RATE_YAW_FILT_HZ     2.5f
#endif

// default gains for Position Controller
#define POSCONTROL_POS_Z_P                    1.0f    // vertical position controller P gain default 1.0
#define POSCONTROL_VEL_Z_P                    5.0f    // vertical velocity controller P gain default 5.0
#define POSCONTROL_ACC_Z_P                    0.5f    // vertical acceleration controller P gain default 0.5
#define POSCONTROL_ACC_Z_I                    0.3f     // vertical acceleration controller I gain default 0.3
#define POSCONTROL_ACC_Z_D                    0.0f    // vertical acceleration controller D gain default 0.0
#define POSCONTROL_ACC_Z_IMAX                 500     // vertical acceleration controller IMAX gain default
#define POSCONTROL_ACC_Z_FILT_HZ              20.0f   // vertical acceleration controller input filter default
#define POSCONTROL_ACC_Z_DT                   0.0025f // vertical acceleration controller dt default
#define POSCONTROL_POS_XY_P                   1.0f    // horizontal position controller P gain default 1.0
#define POSCONTROL_VEL_XY_P                   2.0f    // horizontal velocity controller P gain default 1.6
#define POSCONTROL_VEL_XY_I                   0.4f    // horizontal velocity controller I gain default 0.4
#define POSCONTROL_VEL_XY_D                   0.8f    // horizontal velocity controller D gain default 0.9
#define POSCONTROL_VEL_XY_IMAX                100.0f  // horizontal velocity controller IMAX gain default 100
#define POSCONTROL_VEL_XY_FILT_HZ             5.0f    // horizontal velocity controller input filter default 5.0
#define POSCONTROL_VEL_XY_FILT_D_HZ           5.0f    // horizontal velocity controller input filter for D default 5.0

//UGV attitude control default definition
#define AR_ATTCONTROL_STEER_ANG_P       2.50f
#define AR_ATTCONTROL_STEER_RATE_FF     0.20f
#define AR_ATTCONTROL_STEER_RATE_P      0.20f
#define AR_ATTCONTROL_STEER_RATE_I      0.20f
#define AR_ATTCONTROL_STEER_RATE_IMAX   1.00f
#define AR_ATTCONTROL_STEER_RATE_D      0.00f
#define AR_ATTCONTROL_STEER_RATE_FILT   10.00f
#define AR_ATTCONTROL_STEER_RATE_MAX    360.0f
#define AR_ATTCONTROL_STEER_ACCEL_MAX   180.0f
#define AR_ATTCONTROL_THR_SPEED_P       0.20f
#define AR_ATTCONTROL_THR_SPEED_I       0.20f
#define AR_ATTCONTROL_THR_SPEED_IMAX    1.00f
#define AR_ATTCONTROL_THR_SPEED_D       0.00f
#define AR_ATTCONTROL_THR_SPEED_FILT    10.00f
#define AR_ATTCONTROL_PITCH_THR_P       1.80f
#define AR_ATTCONTROL_PITCH_THR_I       1.50f
#define AR_ATTCONTROL_PITCH_THR_D       0.03f
#define AR_ATTCONTROL_PITCH_THR_IMAX    1.0f
#define AR_ATTCONTROL_PITCH_THR_FILT    10.0f
#define AR_ATTCONTROL_DT                0.02f
#define AR_ATTCONTROL_TIMEOUT_MS        200

// throttle/speed control maximum acceleration/deceleration (in m/s) (_ACCEL_MAX parameter default)
#define AR_ATTCONTROL_THR_ACCEL_MAX     2.00f

// minimum speed in m/s
#define AR_ATTCONTROL_STEER_SPEED_MIN   1.0f

// speed (in m/s) at or below which vehicle is considered stopped (_STOP_SPEED parameter default)
#define AR_ATTCONTROL_STOP_SPEED_DEFAULT    0.1f

// default parameters
#ifndef ROLL_PITCH_YAW_INPUT_MAX
 # define ROLL_PITCH_YAW_INPUT_MAX      45.0f        // roll, pitch and yaw input range
#endif
#ifndef DEFAULT_ANGLE_MAX
 # define DEFAULT_ANGLE_MAX         30.0f            // ANGLE_MAX parameters default value
#endif
#ifndef ANGLE_RATE_MAX
 # define ANGLE_RATE_MAX            180.0f           // default maximum rotation rate in roll/pitch axis requested by angle controller used in stabilize, loiter, rtl, auto flight modes
#endif
#ifndef THR_MIDZ_DEFAULT
 # define THR_MIDZ_DEFAULT           0.1f             // the deadzone above and below mid throttle while in althold
#endif
// default maximum vertical velocity and acceleration the pilot may request
#ifndef PILOT_VELZ_UP_MAX
 # define PILOT_VELZ_UP_MAX              50.0f         // maximum vertical velocity up in cm/s
#endif
#ifndef PILOT_VELZ_DOWN_MAX
 # define PILOT_VELZ_DOWN_MAX              30.0f         // maximum vertical velocity down in cm/s
#endif
#ifndef AUTO_LAND_SPEED
 # define AUTO_LAND_SPEED              30.0f         // maximum vertical velocity down in cm/s
#endif
#ifndef PILOT_ACCEL_Z_DEFAULT
 # define PILOT_ACCEL_Z_DEFAULT        100.0f         // vertical acceleration in cm/s/s while altitude is under pilot control
#endif

#ifndef RANGEFINDER_GAIN_DEFAULT
 # define RANGEFINDER_GAIN_DEFAULT 0.5f     // gain for controlling how quickly rangefinder range adjusts target altitude (lower means slower reaction)
#endif

#ifndef THR_SURFACE_TRACKING_VELZ_MAX
 # define THR_SURFACE_TRACKING_VELZ_MAX 150 // max vertical speed change while surface tracking with rangefinder
#endif

#ifndef RANGEFINDER_TIMEOUT_MS
 # define RANGEFINDER_TIMEOUT_MS  1000      // desired rangefinder alt will reset to current rangefinder alt after this many milliseconds without a good rangefinder alt
#endif

#ifndef RANGEFINDER_GLITCH_ALT_CM
 # define RANGEFINDER_GLITCH_ALT_CM  50.0f      // amount of rangefinder change to be considered a glitch
#endif

#ifndef RANGEFINDER_GLITCH_NUM_SAMPLES
 # define RANGEFINDER_GLITCH_NUM_SAMPLES    50   // number of rangefinder glitches in a row to take new reading
#endif

#ifndef PILOT_TKOFF_ALT_DEFAULT
 # define PILOT_TKOFF_ALT_DEFAULT           60     // default final alt above home for pilot initiated takeoff
#endif

#ifndef LAND_RANGEFINDER_MIN_ALT_CM
 # define LAND_RANGEFINDER_MIN_ALT_CM 		200
#endif

#ifndef LAND_DETECTOR_TRIGGER_SEC
 # define LAND_DETECTOR_TRIGGER_SEC         1.0f    // number of seconds to detect a landing
#endif

#define LAND_CHECK_ANGLE_ERROR_DEG  30.0f       // maximum angle error to be considered landing
#define LAND_CHECK_LARGE_ANGLE_DEG   15.0f     // maximum angle target to be considered landing
#define LAND_CHECK_ACCEL_MOVING     3.0f        // maximum acceleration after subtracting gravity

#define GYRO_INIT_MAX_DIFF_DPS 0.1f

#define THR_HOVER_UPDATE_MIN 0.1f
#define THR_HOVER_UPDATE_MAX 0.7f

// spool definition
// time (in seconds) for throttle to increase from zero to min throttle, and min throttle to full throttle.
#define MOTORS_SPOOL_UP_TIME_DEFAULT 0.5f
#define VIB_LAND_THR 3.0f
#define ACRO_YAW_EXPO 0.0f
#define ACRO_YAW_P 1.5f
#define MAN_THR_FILT_HZ 0.5f
#define LOWBATT_RETURN_VOLT 0.0f
#define LOWBATT_LAND_VOLT 6.6f
#define POSHOLD_VEL_MAX 100.0f		//1m/s
#define POSHOLD_ACCEL_MAX 100.0f	//1m/ss
#define MISSION_VEL_MAX 100.0f		//1m/s
#define MISSION_ACCEL_MAX 100.0f	//1m/ss
#define ALT_RETURN	100.0f			//1m
#define AUTO_TAKEOFF_SPEED 50.0f	//50cm/s
#define LANDING_LOCK_ALT 6.0f
#define FLOW_GAIN_X 1.0f
#define FLOW_GAIN_Y 1.0f
#define FLOW_GAIN_Z 1.0f
#define GNSS_OFFSET_X 0.0f
#define GNSS_OFFSET_Y 0.0f
#define GNSS_OFFSET_Z 0.0f
#define VOLTAGE_GAIN 1.0f
#define CURRENT_GAIN 1.0f
#define UWB_YAW_DELTA_DEG 0.0f
#define UWB_GAIN 1.0f
#define UWB_TAG_ID 1U
#define UWB_TAG_MAX 1U

#define UWB_POS1_X  80.0f
#define UWB_POS1_Y -80.0f
#define UWB_POS1_Z 160.0f

#define UWB_POS2_X   0.0f
#define UWB_POS2_Y 400.0f
#define UWB_POS2_Z 170.0f

#define UWB_POS3_X 400.0f
#define UWB_POS3_Y 400.0f
#define UWB_POS3_Z 170.0f

#define UWB_POS4_X 400.0f
#define UWB_POS4_Y   0.0f
#define UWB_POS4_Z 170.0f

bool arm_motors(void);
void disarm_motors(void);
void lock_motors(void);
void unlock_motors(void);
bool get_soft_armed(void);
void set_soft_armed(bool soft_armed);
bool get_thr_force_decrease(void);
void set_thr_force_decrease(bool force_decrease);
bool get_constrain_vel_d(void);
void set_constrain_vel_d(bool constrain);
void compass_calibrate(void);
bool get_force_autonav(void);
void set_enable_odom(bool enable);
uint8_t get_coordinate_mode(void);

float log_pitch_rad(void);
float log_roll_rad(void);
float log_yaw_rad(void);
float ahrs_pitch_rad(void);					//俯仰角弧度值
float ahrs_roll_rad(void);					//滚转角弧度值
float ahrs_yaw_rad(void);					//偏航角弧度值
float ahrs_pitch_deg(void);					//俯仰角角度值
float ahrs_roll_deg(void);					//滚转角角度值
float ahrs_yaw_deg(void);					//偏航角角度值
float ahrs_cos_roll(void);					//滚转角余弦值
float ahrs_sin_roll(void);					//滚转角正弦值
float ahrs_cos_pitch(void);					//俯仰角余弦值
float ahrs_sin_pitch(void);					//俯仰角正弦值
float ahrs_cos_yaw(void);					//偏航角余弦值
float ahrs_sin_yaw(void);					//偏航角正弦值
const Vector3f& get_accel_ef(void);			//地球坐标系下的三轴加速度
const Vector3f& get_accel_ef_filt(void);	//滤波后地球坐标系下的三轴加速度
const Vector3f& get_gyro_ef(void);			//地球坐标系下的三轴角速度
const Vector3f& get_accel_correct(void);	//修正后的三轴机体加速度
const Vector3f& get_gyro_correct(void);		//修正后的三轴机体角速度
const Vector3f& get_mag_correct(void);		//修正后的三轴磁场强度
const Vector3f& get_accel_filt(void);		//滤波后的三轴机体加速度
const Vector3f& get_gyro_filt(void);		//滤波后的三轴机体角速度
const Vector3f& get_mag_filt(void);			//滤波后的三轴磁场强度
const Matrix3f& get_dcm_matrix(void);		//DCM旋转矩阵
const Matrix3f& get_dcm_matrix_correct(void);
float get_yaw_map(void);

float get_baroalt_filt(void);				//滤波后的气压高度
float get_baro_temp(void);					//控制器温度
float get_rangefinder_alt(void);			//测距仪的测量高度
float get_rangefinder_alt_target(void);		//测距仪的目标高度

void ekf_z_reset(void);
void ekf_xy_reset(void);
Location get_gnss_origin_pos(void); 		//获取系统启动时的初始gnss坐标
Location get_gnss_current_pos(void);		//获取系统当前的gnss坐标
uint8_t get_gnss_reset_notify(void);		//获取航点刷新标志
float get_ned_pos_x(void);//cm
float get_ned_pos_y(void);//cm
float get_ned_pos_z(void);//cm
float get_ned_vel_x(void);//cm/s
float get_ned_vel_y(void);//cm/s
float get_ned_vel_z(void);//cm/s
float get_odom_x(void);//cm
float get_odom_y(void);//cm
float get_odom_z(void);//cm
float get_pos_x(void);//cm
float get_pos_y(void);//cm
float get_pos_z(void);//cm
float get_vel_x(void);//cm/s
float get_vel_y(void);//cm/s
float get_vel_z(void);//cm/s
float get_uwb_x(void);
float get_uwb_y(void);
float get_uwb_z(void);

float get_mav_x_target(void);
float get_mav_y_target(void);
float get_mav_z_target(void);
float get_mav_vx_target(void);
float get_mav_vy_target(void);
float get_mav_vz_target(void);
float get_mav_ax_target(void);
float get_mav_ay_target(void);
float get_mav_az_target(void);
float get_mav_ax_roll_target(void);
float get_mav_ay_pitch_target(void);
float get_mav_yaw_target(void);
float get_mav_yaw_rate_target(void);
bool get_mav_target_state(void);
void reset_mav_target_state(void);
bool use_ego_mission(void);

float get_vib_value(void);
float get_vib_angle_z(void);

bool get_gcs_connected(void);
bool get_offboard_connected(void);
bool get_gnss_location_state(void);
bool has_manual_throttle(void);
void set_manual_throttle(bool manual_throttle);

float get_channel_roll_angle(void);
float get_channel_pitch_angle(void);
float get_channel_yaw_angle(void);

void set_a8mini_yp_rate(int8_t yaw_rate, int8_t pitch_rate, mavlink_channel_t chan);//rate -100~100
void set_a8mini_yp_angle(int16_t yaw_angle, int16_t pitch_angle, mavlink_channel_t chan);//angle*10
void set_a8mini_camera(uint8_t mode, mavlink_channel_t chan);//camera

typedef enum{
	LOG_CAT = 0,
	LOG_DATA,
	LOG_END,
}Log_Type;
typedef enum{
	Logger_Idle = 0,
	Logger_Open,
	Logger_Close,
	Logger_Record,
	Logger_Gnss_Write,
	Logger_Gnss_Read
}Logger_Status;

void Logger_Update(void);
void Logger_Enable(void);
void Logger_Disable(void);
void Logger_Write_Gnss(void);
void Logger_Read_Gnss(void);
void Log_To_File(Log_Type log_type);
void Logger_Cat_Callback(void);
void Logger_Data_Callback(void);

bool mode_althold_init(void);
void mode_althold(void);
bool mode_stabilize_init(void);
void mode_stabilize(void);
bool mode_autonav_init(void);
void mode_autonav(void);
bool mode_poshold_init(void);
void mode_poshold(void);
bool mode_mecanum_init(void);
void mode_mecanum(void);
bool mode_perch_init(void);
void mode_perch(void);
void mode_ugv_a(void);
void mode_ugv_v(void);

/*
 * demo函数声明
 * */
void uwb_send(void);
void uwb_receive(void);
void uwb_range_tx(void);
void uwb_range_rx(void);

// Documentation of GLobals:
typedef union {
  struct {
	  uint8_t unused1                 : 1; // 0
	  uint8_t simple_mode             : 2; // 1,2     // This is the state of simple mode : 0 = disabled ; 1 = SIMPLE ; 2 = SUPERSIMPLE
	  uint8_t pre_arm_rc_check        : 1; // 3       // true if rc input pre-arm checks have been completed successfully
	  uint8_t pre_arm_check           : 1; // 4       // true if all pre-arm checks (rc, accel calibration, gps lock) have been performed
	  uint8_t auto_armed              : 1; // 5       // stops auto missions from beginning until throttle is raised
	  uint8_t logging_started         : 1; // 6       // true if dataflash logging has started
	  uint8_t land_complete           : 1; // 7       // true if we have detected a landing
	  uint8_t new_radio_frame         : 1; // 8       // Set true if we have new PWM data to act on from the Radio
	  uint8_t usb_connected_unused    : 1; // 9       // UNUSED
	  uint8_t rc_receiver_present     : 1; // 10      // true if we have an rc receiver present (i.e. if we've ever received an update
	  uint8_t compass_mot             : 1; // 11      // true if we are currently performing compassmot calibration
	  uint8_t motor_test              : 1; // 12      // true if we are currently performing the motors test
	  uint8_t initialised             : 1; // 13      // true once the init_ardupilot function has completed.  Extended status to GCS is not sent until this completes
	  uint8_t land_complete_maybe     : 1; // 14      // true if we may have landed (less strict version of land_complete)
	  bool 	  throttle_zero           : true; // 15      // true if the throttle stick is at zero, debounced, determines if pilot intends shut-down when not using motor interlock
	  uint8_t system_time_set_unused  : 1; // 16      // true if the system time has been set from the GPS
	  uint8_t gps_glitching           : 1; // 17      // true if GPS glitching is affecting navigation accuracy
	  uint8_t using_interlock         : 1; // 20      // aux switch motor interlock function is in use
	  uint8_t motor_emergency_stop    : 1; // 21      // motor estop switch, shuts off motors when enabled
	  uint8_t land_repo_active        : 1; // 22      // true if the pilot is overriding the landing position
	  uint8_t motor_interlock_switch  : 1; // 23      // true if pilot is requesting motor interlock enable
	  uint8_t in_arming_delay         : 1; // 24      // true while we are armed but waiting to spin motors
	  uint8_t initialised_params      : 1; // 25      // true when the all parameters have been initialised. we cannot send parameters to the GCS until this is done
	  uint8_t compass_init_location   : 1; // 26      // true when the compass's initial location has been set
	  uint8_t rc_override_enable      : 1; // 27      // aux switch rc_override is allowed
	  uint8_t armed_with_switch       : 1; // 28      // we armed using a arming switch
  };
  uint32_t value;
} ap_t;

// Alt_Hold states
enum AltHoldModeState {
	AltHold_MotorStopped,
	AltHold_Takeoff,
	AltHold_Flying,
	AltHold_Landed,
	AltHold_Climb
};

typedef enum {// if add new type, must add to the end of the list
	UINT8,
	UINT16,
	UINT32,
	UINT64,
	FLOAT,
	DOUBLE,
	VECTOR2F,
	VECTOR3F,
	FLOAT_PID,
	FLOAT_PID_2D,
	UINT16_CHANNLE_8
}dataflash_type;

typedef struct {
	bool enabled=false;
	bool alt_healthy=false; // true if we can trust the altitude from the rangefinder
	float alt_cm=0.0f;     // tilt compensated altitude (in cm) from rangefinder
	uint32_t last_healthy_ms=0;
	uint32_t last_update_ms=0;
	LowPassFilterFloat alt_cm_filt; // altitude filter
	int8_t glitch_count;
} Rangefinder_state;
extern Rangefinder_state rangefinder_state;

typedef struct {
	bool healthy;
	float flow_dt;
	uint32_t last_healthy_ms=0;
	Vector2f rads;
	Vector2f vel;
	Vector2f pos;
} Opticalflow_state;
extern Opticalflow_state opticalflow_state;

typedef enum{
	none=0,
	tag=1,
	anchor,
	range,
	comm_tx,
	comm_rx
}uwb_modes;

typedef enum {
	STATE_NONE=0,
	STATE_TAKEOFF,
	STATE_FLYING,
	STATE_FLYING_VIRTUAL,
	STATE_LANDED,
	STATE_STOP,
	STATE_CLIMB, //以上全部为飞行模式的子模式
	STATE_DRIVE
}ROBOT_STATE;
extern ROBOT_STATE robot_state;
extern ROBOT_STATE robot_state_desired;

typedef enum {
	MODE_AIR=0,
	MODE_MECANUM,
	MODE_SPIDER,
	MODE_UGV,
}ROBOT_MAIN_MODE;
extern ROBOT_MAIN_MODE robot_main_mode;

typedef enum {
	MODE_STABILIZE=0,
	MODE_ALTHOLD,
	MODE_POSHOLD,
	MODE_AUTONAV,
	MODE_PERCH,
	MODE_MECANUM_A,
	MODE_MECANUM_V,
	MODE_MECANUM_P,
	MODE_SPIDER_A,
	MODE_SPIDER_P,
	MODE_UGV_A,
	MODE_UGV_V,
	MODE_UGV_P
}ROBOT_SUB_MODE;
extern ROBOT_SUB_MODE robot_sub_mode;

typedef enum {
	MODE_ATTITUDE=0,
	MODE_POSITION,
	MODE_AUTO
}ROBOT_SPEC_MODE;
extern ROBOT_SPEC_MODE robot_spec_mode;

typedef struct{
	// @Param: acro_y_expo
	// @DisplayName: Acro Yaw Expo
	// @Description: Acro yaw expo to allow faster rotation when stick at edges
	// @Values: 0:Disabled,0.1:Very Low,0.2:Low,0.3:Medium,0.4:High,0.5:Very High
	// @Range: -0.5 1.0
	struct acro_y_expo{
		uint16_t num=0;
		dataflash_type type=FLOAT;
		float value=ACRO_YAW_EXPO;
	}acro_y_expo;

	// @Param: acro_yaw_p
	// @DisplayName: Acro Yaw P gain
	// @Description: Converts pilot yaw input into a desired rate of rotation in ACRO, Stabilize and SPORT modes.  Higher values mean faster rate of rotation.
	// @Range: 1 10
	struct acro_yaw_p{
		uint16_t num=1;
		dataflash_type type=FLOAT;
		float value=ACRO_YAW_P;
	}acro_yaw_p;

	// @Param: throttle_midzone
	// @DisplayName: Throttle deadzone
	// @Description: The deadzone above and below mid throttle in PWM microseconds. Used in AltHold, Loiter, PosHold flight modes
	// @Range: 0 0.3
	struct throttle_midzone{
		uint16_t num=2;
		dataflash_type type=FLOAT;
		float value=THR_MIDZ_DEFAULT;
	}throttle_midzone;

	// @DisplayName: Pilot maximum vertical speed descending
	// @Description: The maximum vertical descending velocity the pilot may request in cm/s
	// @Units: cm/s
	// @Range: 5 500
	struct pilot_speed_dn{
		uint16_t num=3;
		dataflash_type type=FLOAT;
		float value=PILOT_VELZ_DOWN_MAX;
	}pilot_speed_dn;

	// @DisplayName: Pilot maximum vertical speed ascending
	// @Description: The maximum vertical ascending velocity the pilot may request in cm/s
	// @Units: cm/s
	// @Range: 50 500
	struct pilot_speed_up{
		uint16_t num=4;
		dataflash_type type=FLOAT;
		float value=PILOT_VELZ_UP_MAX;
	}pilot_speed_up;

	// @DisplayName: Rangefinder gain
	// @Description: Used to adjust the speed with which the target altitude is changed when objects are sensed below the copter
	// @Range: 0.01 2.0
	// @Increment: 0.01
	struct rangefinder_gain{
		uint16_t num=5;
		dataflash_type type=FLOAT;
		float value=RANGEFINDER_GAIN_DEFAULT;
	}rangefinder_gain;

	// @Param: angle_max
	// @DisplayName: Angle Max
	// @Description: Maximum lean angle in all flight modes
	// @Units: degree
	// @Range: 10~80
	struct angle_max{
		uint16_t num=6;
		dataflash_type type=FLOAT;
		float value=DEFAULT_ANGLE_MAX;
	}angle_max;

    // @Param: pilot_accel_z
    // @DisplayName: Pilot vertical acceleration
    // @Description: The vertical acceleration used when pilot is controlling the altitude
    // @Units: cm/s/s
    // @Range: 50 500
    // @Increment: 10
	struct pilot_accel_z{
		uint16_t num=7;
		dataflash_type type=FLOAT;
		float value=PILOT_ACCEL_Z_DEFAULT;
	}pilot_accel_z;

    // @Param: pilot_takeoff_alt
    // @DisplayName: Pilot takeoff altitude
    // @Description: Altitude that altitude control modes will climb to when a takeoff is triggered with the throttle stick.
    // @Units: cm
    // @Range: 0.0 1000.0
	struct pilot_takeoff_alt{
		uint16_t num=8;
		dataflash_type type=FLOAT;
		float value=PILOT_TKOFF_ALT_DEFAULT;
	}pilot_takeoff_alt;

	// @DisplayName: Spool up time
	// @Description: Time in seconds to spool up the motors from zero to min throttle.
	// @Range: 0 2
	// @Units: s
	// @Increment: 0.1
	struct spool_up_time{
		uint16_t num=9;
		dataflash_type type=FLOAT;
		float value=MOTORS_SPOOL_UP_TIME_DEFAULT;
	}spool_up_time;

	// @DisplayName: Throttle filter cutoff
	// @Description: Throttle filter cutoff (Hz) - active whenever altitude control is inactive - 0 to disable
	// @Units: Hz
	// @Range: 0 10
	// @Increment: .5
	struct throttle_filt{
		uint16_t num=10;
		dataflash_type type=FLOAT;
		float value=MAN_THR_FILT_HZ;
	}throttle_filt;

	// @DisplayName: accel_offset
	// @Units: mss
	// @Range: 0 1.0
	struct accel_offsets{
		uint16_t num=11;
		dataflash_type type=VECTOR3F;
		Vector3f value;
	}accel_offsets;

	// @DisplayName: accel_diagonals
	// @Units: mss
	// @Range: 1.0
	struct accel_diagonals{
		uint16_t num=12;
		dataflash_type type=VECTOR3F;
		Vector3f value={1.0f,1.0f,1.0f};
	}accel_diagonals;

	// @DisplayName: accel_offdiagonals
	// @Units: mss
	// @Range: 0 1.0
	struct accel_offdiagonals{
		uint16_t num=13;
		dataflash_type type=VECTOR3F;
		Vector3f value;
	}accel_offdiagonals;

	// @DisplayName: mag_offsets
	// @Units: mGuass
	// @Range: 0 950
	struct mag_offsets{
		uint16_t num=14;
		dataflash_type type=VECTOR3F;
		Vector3f value;
	}mag_offsets;

	// @DisplayName: channel_range
	// @Units: us
	// @Range: 1000 2000
	struct channel_range{
		uint16_t num=15;
		dataflash_type type=UINT16_CHANNLE_8;
		uint16_t channel[8]={
				1100, 	//ch1_min
				1100,	//ch2_min
				1100,	//ch3_min
				1100,	//ch4_min
				1900,	//ch1_max
				1900,	//ch2_max
				1900,	//ch3_max
				1900	//ch4_max
		};
	}channel_range;

	// @DisplayName: auto land speed
	// @Description: vertical descending velocity of the mav in cm/s
	// @Units: cm/s
	// @Range: 5 500
	struct auto_land_speed{
		uint16_t num=16;
		dataflash_type type=FLOAT;
		float value=AUTO_LAND_SPEED;
	}auto_land_speed;

	struct angle_roll_p{
		uint16_t num=17;
		dataflash_type type=FLOAT;
		float value=AC_ATTITUDE_CONTROL_ANGLE_ROLL_P;
	}angle_roll_p;

	struct angle_pitch_p{
		uint16_t num=18;
		dataflash_type type=FLOAT;
		float value=AC_ATTITUDE_CONTROL_ANGLE_PITCH_P;
	}angle_pitch_p;

	struct angle_yaw_p{
		uint16_t num=19;
		dataflash_type type=FLOAT;
		float value=AC_ATTITUDE_CONTROL_ANGLE_YAW_P;
	}angle_yaw_p;

	struct rate_roll_pid{
		uint16_t num=20;
		dataflash_type type=FLOAT_PID;
		float value_p=AC_ATC_MULTI_RATE_ROLL_P;
		float value_i=AC_ATC_MULTI_RATE_ROLL_I;
		float value_d=AC_ATC_MULTI_RATE_ROLL_D;
		float value_imax=AC_ATC_MULTI_RATE_RP_IMAX;
		float value_filt_hz=AC_ATC_MULTI_RATE_RP_FILT_HZ;
	}rate_roll_pid;

	struct rate_pitch_pid{
		uint16_t num=21;
		dataflash_type type=FLOAT_PID;
		float value_p=AC_ATC_MULTI_RATE_PITCH_P;
		float value_i=AC_ATC_MULTI_RATE_PITCH_I;
		float value_d=AC_ATC_MULTI_RATE_PITCH_D;
		float value_imax=AC_ATC_MULTI_RATE_RP_IMAX;
		float value_filt_hz=AC_ATC_MULTI_RATE_RP_FILT_HZ;
	}rate_pitch_pid;

	struct rate_yaw_pid{
		uint16_t num=22;
		dataflash_type type=FLOAT_PID;
		float value_p=AC_ATC_MULTI_RATE_YAW_P;
		float value_i=AC_ATC_MULTI_RATE_YAW_I;
		float value_d=AC_ATC_MULTI_RATE_YAW_D;
		float value_imax=AC_ATC_MULTI_RATE_YAW_IMAX;
		float value_filt_hz=AC_ATC_MULTI_RATE_YAW_FILT_HZ;
	}rate_yaw_pid;

	struct pos_z_p{
		uint16_t num=23;
		dataflash_type type=FLOAT;
		float value=POSCONTROL_POS_Z_P;
	}pos_z_p;

	struct vel_z_p{
		uint16_t num=24;
		dataflash_type type=FLOAT;
		float value=POSCONTROL_VEL_Z_P;
	}vel_z_p;

	struct accel_z_pid{
		uint16_t num=25;
		dataflash_type type=FLOAT_PID;
		float value_p=POSCONTROL_ACC_Z_P;
		float value_i=POSCONTROL_ACC_Z_I;
		float value_d=POSCONTROL_ACC_Z_D;
		float value_imax=POSCONTROL_ACC_Z_IMAX;
		float value_filt_hz=POSCONTROL_ACC_Z_FILT_HZ;
	}accel_z_pid;

	struct pos_xy_p{
		uint16_t num=26;
		dataflash_type type=FLOAT;
		float value=POSCONTROL_POS_XY_P;
	}pos_xy_p;

	struct vel_xy_pid{
		uint16_t num=27;
		dataflash_type type=FLOAT_PID_2D;
		float value_p=POSCONTROL_VEL_XY_P;
		float value_i=POSCONTROL_VEL_XY_I;
		float value_d=POSCONTROL_VEL_XY_D;
		float value_imax=POSCONTROL_VEL_XY_IMAX;
		float value_filt_hz=POSCONTROL_VEL_XY_FILT_HZ;
		float value_filt_d_hz=POSCONTROL_VEL_XY_FILT_D_HZ;
	}vel_xy_pid;

	struct robot_type{
		uint16_t num=28;
		dataflash_type type=UINT8;
		uint8_t value=UAV_4_X;
	}robot_type;

	struct motor_type{
		uint16_t num=29;
		dataflash_type type=UINT8;
		uint8_t value=ESC;
	}motor_type;

	struct update_t_hover_min{
		uint16_t num=30;
		dataflash_type type=FLOAT;
		float value=THR_HOVER_UPDATE_MIN;
	}t_hover_update_min;

	struct update_t_hover_max{
		uint16_t num=31;
		dataflash_type type=FLOAT;
		float value=THR_HOVER_UPDATE_MAX;
	}t_hover_update_max;

	struct vib_land{
		uint16_t num=32;
		dataflash_type type=FLOAT;
		float value=VIB_LAND_THR;
	}vib_land;

	struct horizontal_correct{
		uint16_t num=33;
		dataflash_type type=VECTOR3F;
		Vector3f value={0,0,0};
	}horizontal_correct;

	struct vel_pid_integrator{
		uint16_t num=34;
		dataflash_type type=VECTOR3F;
		Vector3f value={0,0,0};
	}vel_pid_integrator;

	struct rate_pid_integrator{
		uint16_t num=35;
		dataflash_type type=VECTOR3F;
		Vector3f value={0,0,0};
	}rate_pid_integrator;

	struct lowbatt_return_volt{
		uint16_t num=36;
		dataflash_type type=FLOAT;
		float value=LOWBATT_RETURN_VOLT;
	}lowbatt_return_volt;

	struct lowbatt_land_volt{
		uint16_t num=37;
		dataflash_type type=FLOAT;
		float value=LOWBATT_LAND_VOLT;
	}lowbatt_land_volt;

	struct poshold_vel_max{
		uint16_t num=38;
		dataflash_type type=FLOAT;
		float value=POSHOLD_VEL_MAX;
	}poshold_vel_max;

	struct poshold_accel_max{
		uint16_t num=39;
		dataflash_type type=FLOAT;
		float value=POSHOLD_ACCEL_MAX;
	}poshold_accel_max;

	struct mission_vel_max{
		uint16_t num=40;
		dataflash_type type=FLOAT;
		float value=MISSION_VEL_MAX;
	}mission_vel_max;

	struct mission_accel_max{
		uint16_t num=41;
		dataflash_type type=FLOAT;
		float value=MISSION_ACCEL_MAX;
	}mission_accel_max;

	struct alt_return{
		uint16_t num=42;
		dataflash_type type=FLOAT;
		float value=ALT_RETURN;
	}alt_return;

	struct voltage_gain{
		uint16_t num=43;
		dataflash_type type=FLOAT;
		float value=VOLTAGE_GAIN;
	}voltage_gain;

	struct current_gain{
		uint16_t num=44;
		dataflash_type type=FLOAT;
		float value=CURRENT_GAIN;
	}current_gain;

	struct uwb_yaw_delta_deg{
		uint16_t num=45;
		dataflash_type type=FLOAT;
		float value=UWB_YAW_DELTA_DEG;
	}uwb_yaw_delta_deg;

	struct uwb_tag_id{
		uint16_t num=46;
		dataflash_type type=UINT8;
		uint8_t value=UWB_TAG_ID;
	}uwb_tag_id;

	struct uwb_tag_max{
		uint16_t num=47;
		dataflash_type type=UINT8;
		uint8_t value=UWB_TAG_MAX;
	}uwb_tag_max;

	struct uwb_anchor01_pos{
		uint16_t num=48;
		dataflash_type type=VECTOR3F;
		Vector3f value={UWB_POS1_X, UWB_POS1_Y, UWB_POS1_Z};
	}uwb_anchor01_pos;

	struct uwb_anchor02_pos{
		uint16_t num=49;
		dataflash_type type=VECTOR3F;
		Vector3f value={UWB_POS2_X, UWB_POS2_Y, UWB_POS2_Z};
	}uwb_anchor02_pos;

	struct uwb_anchor03_pos{
		uint16_t num=50;
		dataflash_type type=VECTOR3F;
		Vector3f value={UWB_POS3_X, UWB_POS3_Y, UWB_POS3_Z};
	}uwb_anchor03_pos;

	struct uwb_anchor04_pos{
		uint16_t num=51;
		dataflash_type type=VECTOR3F;
		Vector3f value={UWB_POS4_X, UWB_POS4_Y, UWB_POS4_Z};
	}uwb_anchor04_pos;

	struct auto_takeoff_speed{
		uint16_t num=52;
		dataflash_type type=FLOAT;
		float value=AUTO_TAKEOFF_SPEED;
	}auto_takeoff_speed;

	struct baro_temp_offset_gain{
		uint16_t num=53;
		dataflash_type type=FLOAT;
		float value=0.0f;
	}baro_temp_offset_gain;

	struct landing_lock_alt{
		uint16_t num=54;
		dataflash_type type=FLOAT;
		float value=LANDING_LOCK_ALT;
	}landing_lock_alt;

	struct flow_gain{
		uint16_t num=55;
		dataflash_type type=VECTOR3F;
		Vector3f value={FLOW_GAIN_X, FLOW_GAIN_Y, FLOW_GAIN_Z};
	}flow_gain;

	struct gnss_offset{
		uint16_t num=56;
		dataflash_type type=VECTOR3F;
		Vector3f value={GNSS_OFFSET_X, GNSS_OFFSET_Y, GNSS_OFFSET_Z};
	}gnss_offset;

	// @DisplayName: mag_diagonals
	// @Units: mGuass
	// @Range: 1.0
	struct mag_diagonals{
		uint16_t num=57;
		dataflash_type type=VECTOR3F;
		Vector3f value={1.0f,1.0f,1.0f};
	}mag_diagonals;

	// @DisplayName: mag_offdiagonals
	// @Units: mGuass
	// @Range: 0 1.0
	struct mag_offdiagonals{
		uint16_t num=58;
		dataflash_type type=VECTOR3F;
		Vector3f value;
	}mag_offdiagonals;

	struct comm1_bandrate{
		uint16_t num=59;
		dataflash_type type=UINT32;
		uint32_t value=COMM_1_BANDRATE;
	}comm1_bandrate;

	struct comm2_bandrate{
		uint16_t num=60;
		dataflash_type type=UINT32;
		uint32_t value=COMM_2_BANDRATE;
	}comm2_bandrate;

	struct comm3_bandrate{
		uint16_t num=61;
		dataflash_type type=UINT32;
		uint32_t value=COMM_3_BANDRATE;
	}comm3_bandrate;

	struct comm4_bandrate{
		uint16_t num=62;
		dataflash_type type=UINT32;
		uint32_t value=COMM_4_BANDRATE;
	}comm4_bandrate;

	/* *************************************************
	 * ****************Dev code begin*******************/
	// Warning! Developer can add your new code here!
	/* Demo
	 * 此处添加您的自定义参数结构体, e.g:
	struct demo_param_1{
		uint16_t num=401;                 	//参数id号, 为了不影响系统未来更新, 请将您自定义参数的num从401开始
		dataflash_type type=VECTOR3F;	  	//参数类型
		Vector3f value={1.0,1.0,1.0};		//参数默认值,按实际情况自行设置
	}demo_param_1;

	struct demo_param_2{
		uint16_t num=402;                 	//参数id号, id号依次加1, 比如上一个参数id是401, 那么这一个参数id为402
		dataflash_type type=FLOAT;	  		//参数类型
		float value=1.0f;					//参数默认值,按实际情况自行设置
	}demo_param_2;
	 *
	 * */

	/* ****************Dev code end*********************
	 * *************************************************/

}parameter;

extern parameter *param;
