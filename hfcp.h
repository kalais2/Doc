
/**
 *  @file hfcp.h
 *
 *  Copyright (c) 2023, Capgemini - Intelligent Devices
 */

#ifndef _HFCP_H_
#define _HFCP_H_

#include	<stdint.h>

#include	"doip_if.h"

#ifndef PACKED
#define PACKED		__attribute__((packed))
#endif

#define HFCP_MAX_USB_RESP_SIZE		64
#define HID				1
#define MSD				0
#define HFCP_TX_BUFFER_SIZE		64
#define CAN_MSG_DATA_SIZE		8
#define DONT_RELEASE			0
#define DONT_RELEASE_ISR		2
#define RELEASE				1
#define GLOBAL_ADDRESS			0xFF
#define CLAIMED_ADDRESS			0x01
#define PDU_FORMAT_ADDRESS_CLAIM	0xEE
#define TP_CON_MANAGEMENT		0xEC
#define TP_DATA_TRANSFER		0xEB
#define REQUEST_MANAGEMENT		0xEA
#define ACK_MANAGEMENT			0xE8
#define CAN_DATA_LEN_MASK		(0xC000)

/* Number of Channels Used in Garuda */
#define NO_OF_CHANNEL_USED		0x0A	// 4 + 2 (for J1939 protocol
						// CH1 and CH2) + 1 (9141) + 2 CAN-FD

/* Connect Flags */
#define CAN_29BIT_ID			(0x00000100)
#define CAN_ID_BOTH			(0x00000800)
#define ISO9141_K_LINE_ONLY		(0x00001000)
#define ISO9141_NO_CHECKSUM		(0x00000200)

#define HFCP_COM_MODE_DEFAULT	1	
#define HFCP_COM_MODE_USB		1
#define HFCP_COM_MODE_WIFI		2

#define HFCP_TX_BUFFER_SIZE             64
#define HFCP_MAX_IN_SINGLE_FRAME        (1 * HFCP_TX_BUFFER_SIZE)
#define DATALOG_MAX_IN_SINGLE_FRAME     (1 * HFCP_TX_BUFFER_SIZE)
#define MAX_HFCP_RESP_SIZE              (20 * HFCP_MAX_IN_SINGLE_FRAME)
#define SPI_DMA_TX_MAX_SIZE             (MAX_HFCP_RESP_SIZE + 12)
#define HFCP_RX_BUFFER_SIZE             64

#define CANID_FROM_DATA(d) ((((uint8_t *)d)[0] << 24) | (((uint8_t *)d)[1] << 16) | (((uint8_t *)d)[2] << 8) | ((uint8_t *)d)[3])

/* J2534 Error codes */
#define	STATUS_NOERROR			0x00
#define	ERR_NOT_SUPPORTED		0x01
#define	ERR_INVALID_CHANNEL_ID		0x02
#define	ERR_INVALID_PROTOCOL_ID		0x03
#define	ERR_NULL_PARAMETER		0x04
#define	ERR_INVALID_IOCTL_VALUE		0x05
#define	ERR_INVALID_FLAGS		0x06
#define	ERR_FAILED			0x07
#define	ERR_INVALID_DEVICE_ID		0x08
#define	ERR_TIMEOUT			0x09
#define	ERR_INVALID_MSG			0x0A
#define	ERR_INVALID_TIME_INTERVAL	0x0B
#define	ERR_EXCEEDED_LIMIT		0x0C
#define	ERR_INVALID_MSG_ID		0x0D
#define	ERR_DEVICE_IN_USE		0x0E
#define	ERR_INVALID_IOCTL_ID		0x0F
#define	ERR_BUFFER_EMPTY		0x10
#define	ERR_BUFFER_FULL			0x11
#define	ERR_BUFFER_OVERFLOW		0x12
#define	ERR_PIN_INVALID			0x13
#define	ERR_CHANNEL_IN_USE		0x14
#define	ERR_MSG_PROTOCOL_ID		0x15
#define	ERR_INVALID_FILTER_ID		0x16
#define	ERR_NO_FLOW_CONTROL		0x17
#define	ERR_NOT_UNIQUE			0x18
#define	ERR_INVALID_BAUDRATE		0x19
#define	ERR_DEVICE_NOT_CONNECTE		0x1A

struct can_cmd_req {
	uint32_t proto_id;
	uint8_t command;
};

typedef struct can_cmd_req can_cmd_req_t;

typedef enum CAN_COMMAND_ID {
	CAN_EnableComm = 0x2,
	CAN_DisableComm = 0x3,
	CAN_IOCTL_COMMAND = 0x6,
	CAN_Send_msg = 0x08,
	handle_periodic_msg = 0x09,
	CAN_Receive_msg = 0x0A,
	Start_msg_filter = 0x0B,
	Stop_msg_filter = 0x0C,
} CAN_CMD_ID_t;

typedef enum CAN_FD_COMMAND_ID {
	CAN_FD_EnableComm = 0x2,
	CAN_FD_DisableComm = 0x3,
	CAN_FD_IOCTL_COMMAND = 0x6,
	CAN_FD_Send_msg = 0x08,
	handle_FD_periodic_msg = 0x09,
	CAN_FD_Receive_msg = 0x0A,
	Start_FD_msg_filter = 0x0B,
	Stop_FD_msg_filter = 0x0C,
} CAN_FD_CMD_ID_t;

/* CAN Service ids */
#define CAN_EnableComm_ACK		0x02
#define CAN_DisableComm_ACK		0x03
#define CAN_Send_msg_ACK		0x08
#define handle_periodic_msg_ACK		0x09

#define tx_done_CAN_msg			0x0D

/* CAN_FD Service ids */
#define CAN_FD_EnableComm_ACK		0x02
#define CAN_FD_DisableComm_ACK		0x03
#define CAN_FD_Send_msg_ACK		0x08
#define handle_FD_periodic_msg_ACK	0x09

/* KWP Service ids */
#define KWP_EnableComm			0x02
#define KWP_EnableComm_ACK		0x02

#define KWP_DisableComm			0x03
#define KWP_DisableComm_ACK		0x03

#define KWP_Send_msg			0x08
#define KWP_Send_msg_ACK		0x08

#define KWP_Receive_msg			0x0A

/* Transmit and Receive status Flags  */
#define CAN_STD_MSG			0x00000000
#define CAN_EXT_MSG			0x00000100
#define TX_MSG_TYPE_LOOPBACK		0x00000001
#define J1939_ADDRESS_CLAIMED		0x00010000
#define J1939_ADDRESS_LOST		0x00020000

/* J2534 Periodic msg commands */
#define	START_NEW_PERIODIC_MSG_TXN	0x01
#define	UPDATE_DATA_TO_MSG_ID		0x02
#define	STOP_PERIODIC_MSG_TXN		0x03
#define	SUSPEND_ALL_PERIODIC_MSG_TXN	0x04
#define	CHANGE_MSG_PERIODICITY		0x05

/* Filter comands */
#define Start_msg_filter_ACK		0x0B
#define Stop_msg_filter_ACK		0x0C

#define Start_FD_msg_filter_ACK		0x0B
#define Stop_FD_msg_filter_ACK		0x0C

typedef enum HFCP_PROTO_ID {
	SETUP_CMD_ID = 0xC3,
	SESSION_MANAGEMENT = 0xC4,
	J1708_PROTOCOL_ID = 0xC5,
	J1939_PROTOCOL_ID = 0xC6,
	J1939_CH1_PROTOCOL_ID = 0xC7,
	FIRMWARE_UPGRADE_MODE = 0xEA,
	Enable_RNDIS = 0xE1,
	Disable_RNDIS = 0xE0,
	FIRMWARE_SCRIPT = 0xAA,
	TMP_FILE_DELETE = 0xAB,
	CAN_CH1_PROTO_ID = 0x90,
	ISO15765_CH1_PROTO_ID = 0x94,
	CAN_FD_CH1_PROTO_ID = 0x9900,
	ISO15765_FD_CH1_PROTO_ID = 0x91,
	ISO_9141_PROTO_ID = 0x03,
	KWP_PROTOCOL_ID = 0x04,
	CAN_PROTOCOL_ID = 0x05,
	ISO15765_PROTO_ID = 0x06,
	CAN_FD_PS = 0x8011,
	CAN_FD_PROTOCOL_ID = 0x8012,
	ISO15765_FD_PROTO_ID = 0x98,

	DOIP_PROTOCOL_ID = 0x13400,
	UNINITIALIZED_PROTO_ID = 0xFF,
} HFCP_PROTO_ID_t;

typedef enum HFCP_COMMAND_ID {
	Manage_Session = 1,
	DATA_LOG_MODE = 2,
	Setup_FW_Version = 4,
	GET_SERIAL_NO = 5,
	IOCTL_COMMAND = 6,
	Start_Data_logging = 0xF0,
	Stop_Data_logging = 0xF1,
	Data_logging_mode_query = 0xF2,
} HFCP_COMMAND_ID_t;

/* Session Management command Id */
#define SESSION_OPEN			1
#define SESSION_CLOSE			0

#define DATA_LOG_START			1
#define DATA_LOG_STOP			0

/* Setup Command Service ids */
#define Setup_FW_Version_ACK		0x04
#define IOCTL_RESPONSE			0x06

/* IOCTL commands */
#define Get_config		        0x01
#define Get_config_ACK			0x01
#define Set_config			0x02
#define Set_config_ACK			0x02
#define Read_VBATT			0x03
#define Read_VBATT_ACK			0x03
#define KWP_FiveBaudInitialize		0x04
#define KWP_FiveBaudInitialize_ACK	0x04
#define KWP_FastInitialize		0x05
#define KWP_FastInitialize_ACK		0x05
#define clear_all_msg_filters		0x0A
#define clear_all_msg_filters_ACK	0x0A
#define clear_all_periodic_msgs		0x09
#define clear_all_periodic_msgs_ACK	0x09
#define clear_tx_buffer			0x07
#define clear_tx_buffer_ACK		0x07
#define clear_rx_buffer			0x08
#define clear_rx_buffer_ACK		0x08
#define protect_J1939_addr		0xC4
#define protect_J1939_addr_ACK		0xC4

/*IOCTL config parameters*/
#define Get_data_rate			0x01
#define Get_data_rate_ACK		0x01

#define Get_loopback			0x03
#define Get_loopback_ACK		0x03

#define Get_P1_MIN			0x06
#define Get_P1_MIN_ACK			0x06
#define Get_P1_MAX			0x07
#define Get_P1_MAX_ACK			0x07
#define Get_P2_MIN			0x08
#define Get_P2_MIN_ACK			0x08
#define Get_P2_MAX			0x09
#define Get_P2_MAX_ACK			0x09
#define Get_P3_MIN			0x0A
#define Get_P3_MIN_ACK			0x0A
#define Get_P3_MAX			0x0B
#define Get_P3_MAX_ACK			0x0B
#define Get_P4_MIN			0x0C
#define Get_P4_MIN_ACK			0x0C
#define Get_P4_MAX			0x0D
#define Get_P4_MAX_ACK			0x0D

#define Get_sample_point		0x17
#define Get_sample_point_ACK		0x17

#define Get_SJW				0x18
#define Get_SJW_ACK			0x18

#define Get_ps_chan			0x20
#define Get_ps_chan_ACK			0x20

#define Get_frame_length		0x805D
#define Get_frame_length_ACK		0x805D

#define Set_data_rate			0x01
#define Set_data_rate_ACK		0x01

#define Set_loopback			0x03
#define Set_loopback_ACK		0x03

#define Set_P1_MIN			0x06
#define Set_P1_MIN_ACK			0x06

#define Set_P1_MAX			0x07
#define Set_P1_MAX_ACK			0x07

#define Set_P2_MIN			0x08
#define Set_P2_MIN_ACK			0x08

#define Set_P2_MAX			0x09
#define Set_P2_MAX_ACK			0x09

#define Set_P3_MIN			0x0A
#define Set_P3_MIN_ACK			0x0A

#define Set_P3_MAX			0x0B
#define Set_P3_MAX_ACK			0x0B

#define Set_P4_MIN			0x0C
#define Set_P4_MIN_ACK			0x0C

#define Set_P4_MAX			0x0D
#define Set_P4_MAX_ACK			0x0D

#define Set_sample_point		0x17
#define Set_sample_point_ACK		0x17

#define Set_SJW				0x18
#define Set_SJW_ACK			0x18

#define Set_frame_length		0x805D
#define Set_frame_length_ACK		0x805D

#define Set_ps_chan			0x20
#define Set_ps_chan_ACK			0x20

#define Set_fd_datarate			0x21
#define Set_fd_datarate_ACK		0x21

/* sendmsg macro - kwp */
#define SEG_NUM_ZERO       0
#define SEG_NUM_ONE        1

#define APP_BUFFER_SIZE    512



typedef struct {
	uint8_t address[HFCP_MAX_IN_SINGLE_FRAME];
	uint16_t len;
	//uint8_t  freeAddress;
	uint8_t CAN_MsgCount;
} Dynamic_QStruct_t;

/*
 * typedef enum {
 *	CAN_CH1 = 0,
 *	CAN_CH2 = 1
 * } CAN_CH_TypeDef;
 */

typedef enum {
	MID_FAIL,
	MID_PASS
} Mid_API_Status_t;

/*  Type Defines UsedFor J2534 Filters & Periodic Message */

/* Garuda Channel Definitions */

/* The channels need to be re-arranged as GarudaProID_Used[] array entry */
typedef enum {
	GARUDA_CAN_CH1,
	GARUDA_KWP_CH1,
	GARUDA_CAN_CH2,
	GARUDA_J1708,
	GARUDA_J1939_CH1,
	GARUDA_J1939_CH2,
	GARUDA_CAN_FD_CH1,
	GARUDA_CAN_FD_CH2,

	GARUDA_MAX_CH
} GARUDA_ChannelNum_t;

typedef enum {
	GARUDA_IN_DEFAULT_MODE = 0,
	GARUDA_IN_USB_MODE = 1,
	GARUDA_IN_WIFI_MODE = 2,
} GARUDA_CommunicationMode_t;

typedef struct {
	uint32_t protocol_id;
	uint8_t sourceAddr;
	uint8_t nameIdentifier[8];
	uint16_t periodicity;
	uint16_t localperiodicity;
	uint32_t msgId;
} ADDRESS_CLAIM;

typedef struct CAN_EnableComm {
	uint32_t baudrate;
	uint32_t conn_flags;
} PACKED CAN_EnableComm_t;

typedef struct CAN_sendmsg {
	uint8_t seg_num;
	uint16_t dlen;
	uint32_t tx_flags;
	uint32_t canid;
	uint8_t buf[0];
} PACKED CAN_sendmsg_t;

typedef struct CAN_sendmsg_mode1 {
	uint8_t seg_num;
	uint16_t msglen;
	uint8_t conv_id;
	uint32_t tx_flags;
	uint32_t canid;
	uint8_t buf[0];
} PACKED CAN_sendmsg_mode1_t;

typedef struct CAN_sendmsg_mode2 {
	uint8_t seg_num;
	uint16_t msglen;
	uint8_t ack_req;
	uint16_t STMinValue;
	uint8_t buf[0];
} PACKED CAN_sendmsg_mode2_t;

struct mode_data {
	uint32_t canid;
	uint8_t dlen;
	uint8_t buf[0];
} PACKED;

typedef struct CAN_sendmsg_mode3 {
	uint8_t seg_num;
	uint16_t msglen;
	uint32_t tx_flags;
	uint8_t data[0];
} PACKED CAN_sendmsg_mode3_t;

typedef struct lv {
	uint8_t len;
	uint8_t buf[0];
} lv_t;

typedef struct CAN_msg_filter {
	uint8_t filtertype;
	uint8_t buf[0];
} PACKED CAN_msg_filter_t;

typedef struct CAN_stpmsg_filter {
	uint16_t filterid;
} PACKED CAN_stpmsg_filter_t;

typedef struct CAN_periodic_message {
	uint8_t periodic_command;
	uint32_t periodicity;
	uint8_t msgid;
	uint32_t tx_flags;
	uint8_t msglen;
	uint8_t buf[0];
} PACKED CAN_periodic_msg_t;

typedef struct startsession {
	uint8_t startsession;
	uint8_t status;
} PACKED session_t;

typedef struct stopsession {
	uint8_t stopsession;
	uint8_t status;
} PACKED Session_t;

typedef struct IOCTL_request {
	uint32_t command_ID;
	uint8_t length;
	uint32_t data;
	union {
		uint32_t baudrate;
		uint8_t loopback_status;
		uint8_t fd_frame_length;
		uint8_t CAN_FD_PS_chan;
	} un;
} PACKED IOCTL_request_t;


/* KWP structures */
typedef struct KWP_Enablecom {
	uint32_t baudrate;
	uint32_t conn_flags;
}PACKED KWP_Enablecomm_t;

typedef struct KWP_ioctl_req {
	uint8_t command_ID;
	uint8_t length;
	uint8_t param_ID;
	uint32_t data;
}PACKED KWP_ioctl_req_t;

typedef struct KWP_sendmsg {
	uint8_t seg_num;
	uint16_t tx_msg_len;
	uint32_t tx_msg_flags;
	uint8_t buf[512];
}PACKED KWP_sendmsg_t;

typedef struct KWP_msg_filter {
	uint8_t filtertype;
	uint8_t buf[0];
} PACKED KWP_msg_filter_t;

typedef struct KWP_periodic_message {
	uint8_t periodic_command;
	uint32_t periodicity;
	uint8_t msgid;
	uint32_t tx_flags;
	uint8_t msglen;
	uint8_t buf[0];
} PACKED KWP_periodic_msg_t;

typedef struct KWP_stpmsg_filter {
	uint16_t filterid;
} PACKED KWP_stpmsg_filter_t;

struct hfcpReq {
	uint32_t proto_id;	/* See : HFCP_PROTO_ID_t   */
	uint8_t command;	/* See : HFCP_COMMAND_ID_t */
	union {
		session_t startsession;
		Session_t stopsession;
		uint8_t log_mode;
		uint8_t read_vbatt;
		CAN_EnableComm_t can_enable;
		CAN_sendmsg_t can_sndmsg;
		CAN_sendmsg_mode1_t can_sndmsg_m1;
		CAN_sendmsg_mode2_t can_sndmsg_m2;
		CAN_sendmsg_mode3_t can_sndmsg_m3;
		CAN_msg_filter_t can_msg_filter;
		CAN_stpmsg_filter_t can_stpmsg_filter;
		CAN_periodic_msg_t can_periodic_msg;
		IOCTL_request_t ioctl_req;
		doip_cli_cmd_start_session_t start_session;
		doip_cli_cmd_add_doip_server_t doip_sadd;
		doip_cli_cmd_del_doip_server_t doip_sdel;
		doip_cli_cmd_add_node_t doip_addnode;
		doip_cli_cmd_del_node_t doip_delnode;
		doip_cli_cmd_diag_msg_t diagmsg;
		doip_ioctl_handle_t doipioctl;
		doip_client_details_t doipclient;
		KWP_Enablecomm_t kwp_enable;
		KWP_ioctl_req_t kwp_ioctl_req;
		KWP_sendmsg_t kwp_sndmsg;	
		KWP_msg_filter_t kwp_msg_filter;
		KWP_stpmsg_filter_t kwp_stpmsg_filter;
		KWP_periodic_msg_t kwp_periodic_msg;	
	} PACKED u;
} PACKED;

typedef struct hfcpReq hfcpReq_t;

struct hfcp_CAN_Send_msg_resp {
	uint8_t seg_num;
	uint8_t status;
	uint32_t timestamp;
} PACKED;

struct hfcp_msg_filter_resp {
	uint8_t status;
	uint16_t filterid;
} PACKED;

struct hfcp_stpmsg_filter_resp {
	uint8_t status;
	uint16_t filterid;
} PACKED;

struct hfcp_msg_periodic_msg_resp {
	uint8_t status;
	uint16_t per_msg_id;
} PACKED;

struct setup_fw_ver_resp {
	uint8_t status;
	uint8_t buf[0];
} PACKED;

struct read_vbatt_resp {
	uint8_t status;
	uint8_t len;
	uint16_t BatteryVg;
} PACKED;

struct IOCTL_response {
	uint32_t com_id;
	uint8_t status;
	uint8_t length;
	uint32_t data_rate;
	union {
		uint32_t baudrate;
		uint8_t loopback_status;
		uint8_t CAN_FD_PS_chan;
		uint8_t CAN_FD_frame_length;
	} u;
} PACKED;

struct g3_serialno_resp{
	uint8_t status;
	uint8_t buf[0];
} PACKED;

struct doip_big_data {
	uint16_t seqNumm;
	uint16_t lastpktt;
	uint32_t curlen;
} PACKED;
typedef struct doip_big_data doip_big_data_t;

union hfcpResp {
	uint8_t status;
	struct hfcp_CAN_Send_msg_resp sndmsg_rsp;
	struct hfcp_msg_filter_resp msg_fil;
	struct hfcp_stpmsg_filter_resp stpmsg_fil;
	struct hfcp_msg_periodic_msg_resp pmsg;
	struct setup_fw_ver_resp fw_ver;
	struct g3_serialno_resp serialno;
	struct read_vbatt_resp vbatt;
	struct IOCTL_response ioctl_resp;
	union {
		doip_big_data_t d;
		//uint8_t buf[0];
	} u;
	uint8_t buf[0];
} PACKED;
typedef union hfcpResp hfcpResp_t;

/*
 * NOTE: The sizeof 'can_msg_resp_hdr_t' is 64 bytes
 * sizeof(can_msg_resp_hdr) = 4 + (3 * struct can_msg_resp_payload)
 *                          = 61 bytes
 * Hence, if size of any variable is changed then HFCP_MAX_IN_SINGLE_FRAME has to
 * be changed accordingly
 */
struct can_msg_resp_payload {
	uint8_t msg_len;	/* Lenght of rxflags and after it */
	uint16_t rxflags;
	uint32_t timestamp;
	uint32_t msgid;
	uint8_t data[CAN_MSG_DATA_SIZE];
} PACKED;

struct can_msg_resp_hdr {
	uint32_t proto;
	uint8_t command;
	uint8_t seg_num;
	uint8_t mode;		/* b7 - b6 : mode, b5 - b0: msg_count */
	struct can_msg_resp_payload d[1];
} PACKED;

typedef struct can_msg_resp_hdr can_msg_resp_hdr_t;

struct can_msg_resp_data_log_payload {
	uint8_t msg_len;	/* Lenght of rxflags and after it */
	uint16_t rxflags;
	uint8_t timestamp0;
	uint32_t timestamp1;
	uint32_t msgid;
	uint8_t data[CAN_MSG_DATA_SIZE];
} PACKED;

struct can_msg_resp_data_log_hdr {
	uint32_t proto;
	uint8_t mode;		/* b7 - b6 : mode, b5 - b0: msg_count */
	struct can_msg_resp_data_log_payload d[3];
} PACKED;

typedef struct can_msg_resp_hdr can_msg_resp_data_log_hdr_t;

typedef struct {
        uint32_t command;
	uint16_t length;
	uint16_t pkt_number;
	uint8_t seg_number;
	uint16_t index;
	uint8_t buffer[0];
} FirmwareUpgrade_t;

//typedef struct FirmwareUpgrade FirmwareUpgrade_t;

typedef struct {
	uint32_t pkt_number;
	uint8_t is_ready_to_write;
	FirmwareUpgrade_t f_buffer;
} buffer_v;
 

struct FirmwareHost {
	uint32_t command;
	uint16_t length;
	uint8_t seg_number;
	uint16_t pkt_number;
	uint8_t index[0];
} PACKED;

typedef struct FirmwareHost FirmwareHost_t;
int handle_hfcp_rx(uint8_t *buf, int length);

/*
 * Implemented Functions
 */

void HFCP_setApplicationMode(GARUDA_CommunicationMode_t mode);
void Socket_To_Tx(uint8_t sockNum);
void *HFCP_Tx_Task(void *arg);

void Garuda_Tx_data_on_USB(char *pHfcpAck, uint16_t len, uint8_t release);
uint32_t Get_Connect_Flags(uint8_t protocol_id);
void process_CAN_command(uint8_t *buffer);
void process_CAN_FD_command(uint8_t *buffer);
void process_session_mgmt_command(uint8_t *buffer);
void process_Setup_command(uint8_t *buffer);
int Firmware_Update_command(uint8_t *buffer, uint8_t len);

void DisconnectAllChanels(void);
void HFCP_Task_Reset(void);
void HFCP_startTask(void);
void HFCP_pauseTask(void);
void hfcp_disable_CAN_CH1(void);
void hfcp_disable_CAN(void);
int get_CANCH1_or_ISO15756CH1_or_J1939CH1(void);
int get_CAN_or_ISO15756_or_J1939(void);
int get_CAN_FD_or_ISO15756_FD(void);
int get_CAN_FDCH1_or_ISO15765_FDCH1(void);
int process_doip_command(hfcpReq_t *hfcp);

int get_connected_channels(void);
void DisconnectAllChanels(void);
int get_ISO9141_or_14230(void);
int get_J1708_status(void);
int get_KWP_status(void);
int get_CAN_status(void);
int get_CAN_CH1_status(void);
void process_J1939_command(int *buffer);
void J1939_AddressClaim_Global_Response(uint8_t *p_pJ2534Msg,
					uint16_t p_MsgLen,
					GARUDA_ChannelNum_t GarudaChannel);
void suspend_J1939_addressClaim(void);
void AdressClaimHandler(void);
int handle_hfcp_rx(uint8_t *buf, int length);
void process_J1939_command(int *buffer);
ssize_t host_write(const void *,size_t);

#endif /* _HFCP_H_ */
