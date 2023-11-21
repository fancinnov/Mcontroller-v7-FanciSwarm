/*
 * demo_uwb.cpp
 *
 */
#include "maincontroller.h"

/* 定义一个12字节的发送数组:
 * 		数组内容和长度是可以自由定义的，长度最大可以有127个字节，可以按自己的协议逻辑去写，例如：
 *     - byte 0: 以0xC5作为数据包的头.
 *     - byte 1: 表示飞机的id编号.
 *     - byte 2 -> 9: 表示实际想要发送的数据,例如我们想发送DEMOSEND这8个字符.
 *     - byte 10/11: 注意我们自己定义的数据最后两位要空出来，填0即可，它是校验位，由UWB芯片自动生成，不需要我们自己填.  */
static uint8_t tx_msg[12] = {0xC5, 0, 'D', 'E', 'M', 'O', 'S', 'E', 'N', 'D', 0, 0};

//发送数据demo,我们每隔1s发送一条数据tx_msg出去。
void uwb_send(void){
	//获取飞机标签的id编号，该编号可以在app配置
	tx_msg[1]=uwb->TAG_ID;
	//首先我们把发送数据写进UWB芯片的发送缓存
	dwt_writetxdata(sizeof(tx_msg), tx_msg, 0);
	//然后我们设置发送模式，这里注意函数的第3个变量，为0或1，0表示当前数据只是为了通信用，1表示当前数据是通信和测距用，所以配置为1以后uwb芯片会自动记录时间戳。
	dwt_writetxfctrl(sizeof(tx_msg), 0, 1);
	//开始发送，这里填入的变量可以选择立即发送还是延迟发送，demo为立即发送
	dwt_starttx(DWT_START_TX_IMMEDIATE);
	//休眠1s
	osDelay(1000);
}

//接收数据demo
//注意代码一般把常用数据定义为静态变量放在函数外面,为了节省资源
static uint32_t status_reg = 0;
static uint32_t frame_len = 0;
static uint8_t rx_buffer[127];
void uwb_receive(void){
	//首先清空接收数组
	for (uint8_t i = 0 ; i < 127; i++ )
	{
		rx_buffer[i] = 0;
	}
	//立即开启UWB接收
	dwt_rxenable(DWT_START_RX_IMMEDIATE);
	//每隔1ms查询一次UWB的接收标志位，看看有没有接收到数据
	while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
	{osDelay(1);}
	//如果接收到正常数据，就处理数据，否则如果收到了异常数据，就重置UWB
	if (status_reg & SYS_STATUS_RXFCG)
	{
		/* 重置UWB的接收标志位 */
		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);
		/* 获取接收的数据长度. */
		frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
		if (frame_len <= RX_BUF_LEN && frame_len>0)
		{
			//如果接收的数据长度是合理的，就把数据读取出来
			dwt_readrxdata(rx_buffer, frame_len, 0);
		}else{
			//在飞控的USB口输出错误提示语
			usb_printf("UWB read frame_len error!\n");
			return;
		}

		//比对数据的第1个字节是否和协议的包头一致
		if (memcmp(rx_buffer, tx_msg, 1) == 0)
		{
			//打印收到的内容
			usb_printf("id:%d,msg:%c%c%c%c%c%c%c%c\n",rx_buffer[1],rx_buffer[2],rx_buffer[3],rx_buffer[4],rx_buffer[5],rx_buffer[6],rx_buffer[7],rx_buffer[8],rx_buffer[9]);
		}else{
			usb_printf("length:%d\n",frame_len);
			return;
		}
	}else{//收到了异常数据，重置UWB
		/* 清空UWB接收标志位. */
		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
		/* 重置接收. */
		dwt_rxreset();
		//强制关闭UWB的发送和接收
		dwt_forcetrxoff();
	}
}

/* Delay between frames, in UWB microseconds. */
/* This is the delay from the end of the frame transmission to the enable of the receiver, as programmed for the DW1000's wait for response feature. */
#define POLL_TX_TO_RESP_RX_DLY_UUS 50
/* This is the delay from Frame RX timestamp to TX reply timestamp used for calculating/setting the DW1000's delayed TX function. This includes the
 * frame length of approximately 5 ms with above configuration. */
#define RESP_RX_TO_FINAL_TX_DLY_UUS 5000
/* Receive response timeout. See NOTE 5 below. */
#define RESP_RX_TIMEOUT_UUS 20000
#define RELEASE_TIMEOUT 10000

/* Delay between frames, in UWB microseconds. */
/* This is the delay from Frame RX timestamp to TX reply timestamp used for calculating/setting the DW1000's delayed TX function. This includes the
 * frame length of approximately 5 ms with above configuration. */
#define POLL_RX_TO_RESP_TX_DLY_UUS 5000
/* This is the delay from the end of the frame transmission to the enable of the receiver, as programmed for the DW1000's wait for response feature. */
#define RESP_TX_TO_FINAL_RX_DLY_UUS 50
/* Receive final timeout. See NOTE 5 below. */
#define FINAL_RX_TIMEOUT_UUS 20000
/* This is the delay from now to send message*/
#define TIME_TX_DLY_UUS 4000

/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion factor.
 * */
#define UUS_TO_DWT_TIME 65536
/* Default antenna delay values for 64 MHz PRF. See NOTE 1 below. */
#define TX_ANT_DLY 16485
#define RX_ANT_DLY 16485

/* Frames used in the ranging process. */
	/* The frame sent in this example is an 802.15.4e standard blink.
	 * It is a more than 8 bytes frame composed of the following fields:
	 *     - byte 0/1: header (0x41 0x88).
	 *     - byte 2: sequence number, incremented for each new frame.
	 *     - byte 3: tag ID.
	 *     - byte 4: unused.
	 *     - byte 5: frame type.
	 *     - middle bytes: data.
	 *     - last two bytes: frame check-sum, automatically set by DW1000.  */
#define ALL_MSG_SN_IDX 2
#define ALL_MSG_TAG_IDX 3
#define FINAL_MSG_POLL_TX_TS_IDX 6
#define FINAL_MSG_RESP_RX_TS_IDX 10
#define FINAL_MSG_FINAL_TX_TS_IDX 14
#define FINAL_MSG_TS_LEN 4
/* Length of the common part of the message (up to and including the function code. */
#define ALL_MSG_COMMON_LEN 6
static	uint8_t rx_poll_msg[8] =  	{0x41, 0x88, 0, 0x0, 0xDE, 0x21, 0, 0};
static	uint8_t tx_resp_msg[11] =  	{0x41, 0x88, 0, 0x0, 0xDE, 0x10, 0x02, 0, 0, 0, 0};
static	uint8_t rx_final_msg[20] = 	{0x41, 0x88, 0, 0x0, 0xDE, 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static	uint8_t distance_msg[17] = 	{0x41, 0x88, 0, 0x0, 0xDE, 0xAA, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static	uint8_t tx_poll_msg[8] =  	{0x41, 0x88, 0, 0x0, 0xDE, 0x21, 0, 0};
static	uint8_t rx_resp_msg[11] =  	{0x41, 0x88, 0, 0x0, 0xDE, 0x10, 0x02, 0, 0, 0, 0};
static	uint8_t tx_final_msg[20] = 	{0x41, 0x88, 0, 0x0, 0xDE, 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

/* Frame sequence number, incremented after each transmission. */
static	uint8_t frame_seq_nb = 0;
/* Timestamps of frames transmission/reception.
 * As they are 40-bit wide, we need to define a 64-bit int type to handle them. */
static	uint64_t poll_rx_ts;
static	uint64_t resp_tx_ts;
static	uint64_t final_rx_ts;

static	uint64_t poll_tx_ts_tag;
static	uint64_t resp_rx_ts_tag;
static	uint64_t final_tx_ts_tag;

static	uint32_t final_tx_time;
static  uint32_t resp_tx_time;
static	uint32_t poll_tx_ts, resp_rx_ts, final_tx_ts;
static	uint32_t poll_rx_ts_32, resp_tx_ts_32, final_rx_ts_32;
static	double Ra, Rb, Da, Db;
static	double tof_dtu;
static  double tof;
static	double distance_temp;
static	uint8_t uwb_state=0;
static int temp;
enum{
	idle=0,
	receive,
	poll,
	resp,
	final,
	dis,
	release,
	release_confirm,
	release_wait,
	comm,
	waiting,
	ranging
};

//测距发送端demo
void uwb_range_tx(void){
	switch(uwb_state){
	case idle:
		osDelay(1000);
		/* Set expected response's delay and timeout.
		 * As this example only handles one incoming frame with always the same delay and timeout, those values can be set here once for all. */
		dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
		dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
		uwb_state=poll;
	case poll:
		/* Write frame data to DW1000 and prepare transmission. */
		tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
		tx_poll_msg[ALL_MSG_TAG_IDX] = uwb->TAG_ID;
		dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0);
		dwt_writetxfctrl(sizeof(tx_poll_msg), 0, 1);
		/* Start transmission, indicating that a response is expected so that reception is enabled automatically after the frame is sent and the delay
		 * set by dwt_setrxaftertxdelay() has elapsed. */
		dwt_starttx(DWT_START_TX_IMMEDIATE| DWT_RESPONSE_EXPECTED);
		uwb_state=resp;
	case resp:
		/* We assume that the transmission is achieved correctly, poll for reception of a frame or error/timeout. */
		while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
		{osDelay(1);}
		if (status_reg & SYS_STATUS_RXFCG)
		{
			/* Clear good RX frame event and TX frame sent in the DW1000 status register. */
			dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);
			/* A frame has been received, read it into the local buffer. */
			frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
			if (frame_len <= RX_BUF_LEN && frame_len>0)
			{
				dwt_readrxdata(rx_buffer, frame_len, 0);
			}else{
				/* Activate reception immediately. */
				dwt_rxenable(DWT_START_RX_IMMEDIATE);
//				usb_printf("UWB read frame_len error!\n");
				break;
			}

			/* As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
			rx_buffer[ALL_MSG_TAG_IDX] = 0;
			rx_buffer[ALL_MSG_SN_IDX] = 0;

			if (memcmp(rx_buffer, rx_resp_msg, ALL_MSG_COMMON_LEN) == 0)
			{
				uwb_state=final;
			}else{
				/* Activate reception immediately. */
				dwt_rxenable(DWT_START_RX_IMMEDIATE);
				break;
			}
		}else{
			/* Clear RX error/timeout events in the DW1000 status register. */
			dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);

			/* Reset RX to properly reinitialise LDE operation. */
			dwt_rxreset();
			dwt_forcetrxoff();//重置UWB通信
			uwb_state=idle;
			break;
		}
	case final:
		/* Retrieve poll transmission and response reception timestamp. */
		poll_tx_ts_tag = uwb->get_tx_timestamp_u64();
		resp_rx_ts_tag = uwb->get_rx_timestamp_u64();

		/* Compute final message transmission time. */
		final_tx_time = (resp_rx_ts_tag + (RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
		dwt_setdelayedtrxtime(final_tx_time);

		/* Final TX timestamp is the transmission time we programmed plus the TX antenna delay. */
		final_tx_ts_tag = ((((uint64_t)final_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

		/* Write all timestamps in the final message. See NOTE 10 below. */
		uwb->final_msg_set_ts(&tx_final_msg[FINAL_MSG_POLL_TX_TS_IDX], poll_tx_ts_tag);
		uwb->final_msg_set_ts(&tx_final_msg[FINAL_MSG_RESP_RX_TS_IDX], resp_rx_ts_tag);
		uwb->final_msg_set_ts(&tx_final_msg[FINAL_MSG_FINAL_TX_TS_IDX], final_tx_ts_tag);

		/* Write and send final message. */
		tx_final_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
		tx_final_msg[ALL_MSG_TAG_IDX] = uwb->TAG_ID;
		dwt_writetxdata(sizeof(tx_final_msg), tx_final_msg, 0);
		dwt_writetxfctrl(sizeof(tx_final_msg), 0, 1);
		//TODO maybe need longer time
		if (dwt_starttx(DWT_START_TX_DELAYED|DWT_RESPONSE_EXPECTED) == DWT_ERROR)
		{
//			usb_printf("UWB tx error!\n");
			uwb_state=idle;
			break;
		}
		uwb_state=dis;
		/* Clear reception timeout to start next ranging process. */
		dwt_setrxtimeout(RELEASE_TIMEOUT);
	case dis:
		/* We assume that the transmission is achieved correctly, poll for reception of a frame or error/timeout. */
		while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
		{
			osDelay(1);
		}
		/* Increment frame sequence number after transmission of the poll message. */
		if (status_reg & SYS_STATUS_RXFCG)
		{
			/* Clear good/fail RX frame event in the DW1000 status register. */
			dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);
			/* A frame has been received, read it into the local buffer. */
			frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;

			if (frame_len <= RX_BUF_LEN && frame_len>0)
			{
				dwt_readrxdata(rx_buffer, frame_len, 0);
			}else{
//					usb_printf("UWB read frame_len error!\n");
				/* Clear reception timeout to start next ranging process. */
				dwt_setrxtimeout(0);
				/* Activate reception immediately. */
				dwt_rxenable(DWT_START_RX_IMMEDIATE);
				break;
			}

			/*As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
			rx_buffer[ALL_MSG_TAG_IDX] = 0;
			rx_buffer[ALL_MSG_SN_IDX] = 0;

			if (memcmp(rx_buffer, distance_msg, ALL_MSG_COMMON_LEN) == 0)
			{
				distance_temp=(rx_buffer[6]*100 + rx_buffer[7]);
				usb_printf("distance:%f cm \n",distance_temp);
				frame_seq_nb++;
			}else{
				/* Clear reception timeout to start next ranging process. */
				dwt_setrxtimeout(0);
				/* Activate reception immediately. */
				dwt_rxenable(DWT_START_RX_IMMEDIATE);
				break;
			}
		}else{
			/* Clear RX error/timeout events in the DW1000 status register. */
			dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
			/* Reset RX to properly reinitialise LDE operation. */
			dwt_rxreset();
			dwt_forcetrxoff();//重置UWB通信
			uwb_state=idle;
			break;
		}
		uwb_state=idle;
	default:
		uwb_state=idle;
		break;
	}
}

//测距接收端demo
void uwb_range_rx(void){
	switch(uwb_state){
	case idle:
		/* Clear reception timeout to start next ranging process. */
		dwt_setrxtimeout(0);
		/* Activate reception immediately. */
		dwt_rxenable(DWT_START_RX_IMMEDIATE);
		uwb_state=receive;
	case receive:
		/* Poll for reception of a frame or error/timeout. */
		while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
		{
			osDelay(1);
		}
		if (status_reg & SYS_STATUS_RXFCG)
		{
			/* Clear good RX frame event in the DW1000 status register. */
			dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);

			/* A frame has been received, read it into the local buffer. */
			frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
			if (frame_len <= RX_BUFFER_LEN && frame_len>0)
			{
				dwt_readrxdata(rx_buffer, frame_len, 0);
			}else{
//				usb_printf("UWB read frame_len error!\n");
				/* Clear reception timeout to start next ranging process. */
				dwt_setrxtimeout(0);
				/* Activate reception immediately. */
				dwt_rxenable(DWT_START_RX_IMMEDIATE);
				break;
			}
			/* Check that the frame is a poll sent by "DS TWR initiator" example.
			 * As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
			frame_seq_nb = rx_buffer[ALL_MSG_SN_IDX];

			rx_buffer[ALL_MSG_SN_IDX] = 0;
			rx_buffer[ALL_MSG_TAG_IDX] = 0;
			if (memcmp(rx_buffer, rx_poll_msg, ALL_MSG_COMMON_LEN) == 0){
				/* Retrieve poll reception timestamp. */
				poll_rx_ts = uwb->get_rx_timestamp_u64();

				/* Set send time for response. See NOTE 9 below. */
				resp_tx_time = (poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
				dwt_setdelayedtrxtime(resp_tx_time);

				/* Set expected delay and timeout for final message reception. See NOTE 4 and 5 below. */
				dwt_setrxaftertxdelay(RESP_TX_TO_FINAL_RX_DLY_UUS);
				dwt_setrxtimeout(0);

				uwb_state=resp;
				break;
			}else if (memcmp(rx_buffer, rx_final_msg, ALL_MSG_COMMON_LEN) == 0){
				uwb_state=dis;
				break;
			}else{
				/* Reset RX to properly reinitialise LDE operation. */
				dwt_rxreset();
				dwt_forcetrxoff();//重置UWB通信
				uwb_state=idle;
				break;
			}
		}else{
			/* Clear RX error/timeout events in the DW1000 status register. */
			dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);

			/* Reset RX to properly reinitialise LDE operation. */
			dwt_rxreset();
			dwt_forcetrxoff();//重置UWB通信
			uwb_state=idle;
			break;
		}
	case resp:
		/* Write and send the response message. See NOTE 9 below.*/
		tx_resp_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
		tx_resp_msg[ALL_MSG_TAG_IDX] = uwb->TAG_ID;
		dwt_writetxdata(sizeof(tx_resp_msg), tx_resp_msg, 0);
		dwt_writetxfctrl(sizeof(tx_resp_msg), 0, 1);

		/* If dwt_starttx() returns an error, abandon this ranging exchange and proceed to the next one. */
		if (dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED) == DWT_ERROR)
		{
//			usb_printf("UWB tx error!\n");
			uwb_state=idle;
			break;
		}
		uwb_state=receive;
		break;
	case dis:
		/* Retrieve response transmission and final reception timestamps. */
		resp_tx_ts = uwb->get_tx_timestamp_u64();
		final_rx_ts = uwb->get_rx_timestamp_u64();

		/* Get timestamps embedded in the final message. */
		uwb->final_msg_get_ts(&rx_buffer[FINAL_MSG_POLL_TX_TS_IDX], &poll_tx_ts);
		uwb->final_msg_get_ts(&rx_buffer[FINAL_MSG_RESP_RX_TS_IDX], &resp_rx_ts);
		uwb->final_msg_get_ts(&rx_buffer[FINAL_MSG_FINAL_TX_TS_IDX], &final_tx_ts);
		if( poll_rx_ts==0||
			poll_tx_ts==0||
			resp_tx_ts==0||
			resp_rx_ts==0||
			final_rx_ts==0||
			final_tx_ts==0){
			uwb_state=idle;
			break;
		}
		/* Compute time of flight. 32-bit subtractions give correct answers even if clock has wrapped. */
		poll_rx_ts_32 = (uint32_t)poll_rx_ts;
		resp_tx_ts_32 = (uint32_t)resp_tx_ts;
		final_rx_ts_32 = (uint32_t)final_rx_ts;

		Ra = (double)(resp_rx_ts - poll_tx_ts);
		Rb = (double)(final_rx_ts_32 - resp_tx_ts_32);
		Da = (double)(final_tx_ts - resp_rx_ts);
		Db = (double)(resp_tx_ts_32 - poll_rx_ts_32);
		tof_dtu = (Ra * Rb - Da * Db) / (Ra + Rb + Da + Db);
		tof = tof_dtu * DWT_TIME_UNITS;
		distance_temp = tof * SPEED_OF_LIGHT;
		distance_temp = distance_temp - dwt_getrangebias(uwb->config.chan,(float)distance_temp, uwb->config.prf);
		usb_printf("dis: %f m \n", distance_temp);
		temp = (int)(distance_temp*100); //cm,解算距离
		distance_msg[6] = temp/100;
		distance_msg[7] = temp%100;
		distance_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
		distance_msg[ALL_MSG_TAG_IDX] = uwb->TAG_ID;
		dwt_writetxdata(sizeof(distance_msg), distance_msg, 0);
		dwt_writetxfctrl(sizeof(distance_msg), 0, 1);
		poll_rx_ts=0;
		poll_tx_ts=0;
		resp_tx_ts=0;
		resp_rx_ts=0;
		final_rx_ts=0;
		final_tx_ts=0;

		if (dwt_starttx(DWT_START_TX_IMMEDIATE) == DWT_ERROR)
		{
			osThreadYield();
			break;
		}
		uwb_state=idle;
		break;
	default:
		uwb_state=idle;
		break;
	}
}

