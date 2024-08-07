/*
 * uwb.h
 *
 *  Created on: Nov 20, 2021
 *      Author: 25053
 */
#pragma once

#ifndef INCLUDE_UWB_UWB_H_
#define INCLUDE_UWB_UWB_H_

#include "hal.h"
#include "uwb/deca_device_api.h"
#include "uwb/deca_regs.h"
#include "uwb/trilateration.h"
#include "common.h"
#ifdef __cplusplus

#define RX_BUF_LEN 127
#define UWB_RINGBUFFER_LEN 2048

class UWB{
public:
	/// Constructor
	UWB();
	/* Default communication configuration. We use here EVK1000's default mode (mode 3). */
	dwt_config_t config =
	{
		5,               /* Channel number. */
		DWT_PRF_64M,     /* Pulse repetition frequency. */
		DWT_PLEN_1024,   /* Preamble length. */
		DWT_PAC32,       /* Preamble acquisition chunk size. Used in RX only. */
		9,               /* TX preamble code. Used in TX only. */
		9,               /* RX preamble code. Used in RX only. */
		1,               /* Use non-standard SFD (Boolean) */
		DWT_BR_110K,     /* Data rate. */
		DWT_PHRMODE_STD, /* PHY header mode. */
		(1025 + 64 - 32) /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
	};
	bool get_uwb_position=false;
	bool get_uwb_distance=false;
	int Anchordistance[4]={0};
	uint16_t Anchorvolt[4]={0};
	int Tagdistance[127]={0};
	Vector3f uwb_position;
	float* distance_tag;
	uint8_t TAG_ID=1;
	bool uwb_init(void);
	void uwb_update(void);
	void config_uwb(uwb_modes mode, uint8_t id, uint8_t tag_master, uint8_t tag_start, uint8_t tag_max, uint8_t anchor_max);
	void set_anchor_positon(uint8_t id, double x, double y, double z);
	uint64_t get_sys_timestamp_u64(void);
	uint64_t get_tx_timestamp_u64(void);
	uint64_t get_rx_timestamp_u64(void);
	void final_msg_get_ts(const uint8_t *ts_field, uint32_t *ts);
	void final_msg_set_ts(uint8_t *ts_field, uint32_t ts);
	float get_range_distance(uint8_t tag_a_id, uint8_t tag_b_id);
	RingBuffer ringbuffer_uwb_rx, ringbuffer_uwb_tx;
	void send_buff(uint8_t* buf, uint16_t len);

private:

	typedef enum{
		idle=0,
		receive,
		poll,
		resp,
		final,
		dis,
		release,
		release_confirm,
		release_wait,
		tx,
		rx,
		waiting,
		ranging
	}uwb_states;

	/* Frames used in the ranging process. See NOTE 2 below. */
	/* The frame sent in this example is an 802.15.4e standard blink.
	 * It is a more than 8 bytes frame composed of the following fields:
	 *     - byte 0/1: header (0x41 0x88).
	 *     - byte 2: sequence number, incremented for each new frame.
	 *     - byte 3: tag ID.
	 *     - byte 4: unused.
	 *     - byte 5: frame type.
	 *     - middle bytes: data.
	 *     - last two bytes: frame check-sum, automatically set by DW1000.  */
	uint8_t rx_poll_msg[8] =  	{0x41, 0x88, 0, 0x0, 0xDE, 0x21, 0, 0};
	uint8_t tx_resp_msg[11] =  	{0x41, 0x88, 0, 0x0, 0xDE, 0x10, 0x02, 0, 0, 0, 0};
	uint8_t rx_final_msg[20] = 	{0x41, 0x88, 0, 0x0, 0xDE, 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	uint8_t distance_msg[17] = 	{0x41, 0x88, 0, 0x0, 0xDE, 0xAA, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	uint8_t tx_poll_msg[8] =  	{0x41, 0x88, 0, 0x0, 0xDE, 0x21, 0, 0};
	uint8_t rx_resp_msg[11] =  	{0x41, 0x88, 0, 0x0, 0xDE, 0x10, 0x02, 0, 0, 0, 0};
	uint8_t tx_final_msg[20] = 	{0x41, 0x88, 0, 0x0, 0xDE, 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	uint8_t time_msg[13] =  	{0x41, 0x88, 0, 0x0, 0xDE, 0x36, 0, 0, 0, 0, 0, 0, 0};
	uint8_t angle_msg[27] =    	{0x41, 0x88, 0, 0x0, 0xDE, 0xFE, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	uint8_t Semaphore_Release[9] =    				{0x41, 0x88, 0, 0x0, 0xDE, 0xE0, 0, 0, 0};
	uint8_t Tag_Statistics[9] =                   	{0x41, 0x88, 0, 0x0, 0xDE, 0xE1, 0, 0, 0};
	uint8_t Master_Release_Semaphore[9] =         	{0x41, 0x88, 0, 0x0, 0xDE, 0xE2, 0, 0, 0};
	uint8_t Tag_Statistics_response[9] =          	{0x41, 0x88, 0, 0x0, 0xDE, 0xE3, 0, 0, 0};
	uint8_t Master_Release_Semaphore_comfirm[9] = 	{0x41, 0x88, 0, 0x0, 0xDE, 0xE4, 0, 0, 0};
	uint8_t* tx_final_msg_prt;
	uint16_t tx_final_msg_prt_size=0;

	/* Frame sequence number, incremented after each transmission. */
	uint8_t frame_seq_nb = 0;

	/* Buffer to store received messages.
	 * Its size is adjusted to longest frame that this example code is supposed to handle. */
	uint8_t rx_buffer[RX_BUF_LEN];

	/* Hold copy of status register state here for reference, so reader can examine it at a breakpoint. */
	uint32_t status_reg = 0;

	/* Timestamps of frames transmission/reception.
	 * As they are 40-bit wide, we need to define a 64-bit int type to handle them. */
	uint64_t poll_rx_ts;
	uint64_t resp_tx_ts;
	uint64_t final_rx_ts;
	uint64_t resp_rx_ts_range;

	uint64_t poll_tx_ts_tag;
	uint64_t resp_rx_ts_tag;
	uint64_t final_tx_ts_tag;

	uint64_t* resp_rx_ts_prt;

	/* Hold copies of computed time of flight and distance here for reference, so reader can examine it at a breakpoint. */
	double tof;
	double distance;
	double distance_temp;
	double Kg=1.0f;

	/* String used to display measured distance on LCD screen (16 characters maximum). */
	char dist_str[16] = {0};

	void dwt_dumpregisters(char *str, size_t strSize);
	void Anchor_Array_Init(void);
	void Semaphore_Init(void);
	int Sum_Tag_Semaphore_request(void);
	/* Declaration of static functions. */
	void compute_angle_send_to_anchor0(int distance1, int distance2, int distance3);
	void distance_mange(void);

	uint8_t MASTER_TAG=1;
	uint8_t MAX_SLAVE_TAG=1;
	uint8_t SLAVE_TAG_START_INDEX=1;
	uint8_t ANCHOR_MAX_NUM=4; //3 4
	uint8_t ANCHOR_IND=1;  // 1 2 3 4

	uint8_t Semaphore[255];
	vec3d AnchorList[4];
	vec3d tag_best_solution;
	float L[3]={0.0f};
	float A[3*3]={0.0f};
	float A_INV[3*3]={0.0f};

	uint8_t anchor_index = 0;
	uint8_t tag_index = 0;
	uint8_t tag_flag = 0;

	uint8_t Semaphore_Enable = 0 ;
	uint8_t Waiting_TAG_Release_Semaphore = 0;
	uint32_t frame_len = 0;
	uint32_t tag_time=0,wait_time=0,range_time=0;
	uint8_t last_tag=0;
	uwb_states uwb_state=idle;
	uwb_modes uwb_mode=none;

	bool init_uwb_tag=false, get_dis=false;
	uint32_t final_tx_time;
	uint32_t poll_tx_ts, resp_rx_ts, final_tx_ts;
	uint32_t poll_rx_ts_32, resp_tx_ts_32, final_rx_ts_32;
	double Ra, Rb, Da, Db;
	double tof_dtu;
	uint32_t resp_tx_time;
	int ret;
	int temp;
	uint32_t delta_ts;
	uint32_t delta_ts_anchor[4];
	uint8_t delta_ts_anchor0_num=0;
	uint8_t delta_ts_anchor1_num=0;
	uint8_t delta_ts_anchor2_num=0;
	uint8_t delta_ts_anchor3_num=0;
	bool get_all_tag_delta_ts=false;
	uint16_t volt=0;
	uint8_t tx_msg[RX_BUF_LEN];
	uint8_t TxBuffer[UWB_RINGBUFFER_LEN],RxBuffer[UWB_RINGBUFFER_LEN];
};

#endif
#endif /* INCLUDE_UWB_UWB_H_ */
