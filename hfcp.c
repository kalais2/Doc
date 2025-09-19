
/**
 *  @file hfcp.c
 *
 *  Copyright (c) 2023-2024, Capgemini - Intelligent Devices
 */

#include <stdio.h>

#include	"can.h"
#include	"j2534_filter.h"
#include	"j2534_periodic.h"
#include	"data_logger.h"

#include	"hfcp.h"
#include	"g3d.h"
#include	"net_if.h"
#include	"usbhid.h"
#include	"led.h"
//#include        "kwp_if.h"

/* TODO: Move this to Makefile */
#define GARUDA_USB_DEBUG    0
#define USB_PKT_SIZE       (512)
#define CAN_DL_MIN_SIZE(a, b) (((a) < (b)) ? (a) : (b))
#define IN_BUFFER_SIZE 512

char *CAN_CHA1 = "can0";
char *CAN_CHA2 = "can1";
extern uint8_t USB_Device_Mode;
static uint8_t checkopendevice;
#if GARUDA_USB_DEBUG
extern Garuda_Debug_Info Garuda_Debug;
extern Garuda_Debug_Info Garuda_Debug_CAN2;
#endif
uint8_t HFCPComMode;
uint32_t GarudaProID_Used[NO_OF_CHANNEL_USED] = { 0x0005, 0x0004, 0x0090, 0x00C5, 0x00C6, 0x00C7, 0x8012, 0x9900, 0x0030, 0x0000 };
uint8_t timestamp_id[NO_OF_CHANNEL_USED] =
    { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
uint8_t AdressClaimInitiated = 0;
uint8_t AdressClaimStatus = 0;
uint8_t device_claimed_Address = 0xFE;
ADDRESS_CLAIM AdressClaim;
uint8_t l_connected_channels = 0;

uint8_t l_connected_channel_1 = 0;
uint8_t l_connected_channel_2 = 0;
uint8_t l_connected_channel_Kline = 0;
uint8_t l_connected_channel_J1939_1 = 0;
uint8_t l_connected_channel_J1939_2 = 0;

static uint8_t CAN_FD_PS_chan = 1;
static uint8_t CAN_active = 0;
static uint8_t CAN_CH1_active = 0;

static uint8_t KWP_active               = 0;
//static uint8_t J1708_active             = 0;
static uint8_t CAN_or_ISO = 0;
static uint8_t CAN1_or_ISO1 = 0;
static uint8_t CAN_or_ISO15765_J1939 = 0;
static uint32_t CAN_FD_or_ISO15765_FD = 0;
static uint8_t CANCH1_or_ISO15765CH1_J1939CH1 = 0;
static uint32_t CAN_FDCH1_or_ISO15765_FDCH1 = 0;
static uint32_t CAN_connect_flags = 0;

//static uint32_t KWP_connect_flags       = 0;
static uint8_t ISO_9141_OR_14230        = 0;

static uint32_t CAN_CH1_connect_flags = 0;
static uint32_t stored_msg_id_CAN = 0;
static uint32_t stored_tx_flags_CAN = 0;
static uint32_t stored_msg_id_CAN_CH1 = 0;
static uint8_t fd_frame_length = 64;

//static uint32_t stored_tx_flags_CAN_CH1    = 0;
//static uint8_t stored_is_ext_frm           = 0; // FALSE;
//static GARUDA_CommunicationMode_t appMode;

//static uint16_t hfcp_max_in_single_frame;
//static uint16_t hfcp_max_resp_size;

/* ECU_SendMessage shall use this buffer */
static ISO9141_14230_TxMsg_S l_App_ISO9141_14230TxMsg_S; // kwp_ch

/* Flags to Indicate the Segment Transfer */
static uint8_t l_KWPTX_SegTrnsfr=0;
static uint8_t hfcpTxSock;

uint16_t buff_size_in_MB = 0;

uint8_t CAN1_Vehicle_STMinValue = 0;
uint8_t CAN2_vehicle_STMinValue = 0;

//size_t msg_lenth;
#if 0
mqd_t xHFCP_TX_Queue;
mqd_t mqd_HFCP_REC;
mqd_t mqd;
mqd_t que_ret1;
mqd_t que_ret2;
#endif
//uint8_t hfcpAckFrame[3];

uint8_t get_CAN_or_ISO(void);
uint8_t get_CAN1_or_ISO1(void);

static void process_CAN_IOCTL_cmd(uint8_t *);
static void process_CAN_FD_IOCTL_cmd(uint8_t *);

static buffer_v buffer_t;
static void process_KWP_IOCTL_cmd(uint8_t *buffer);  

//ISO9141_14230_TxMsg_S ISO9141_14230_TxMsg_S_Buffer;/* it is extern variable .declaration in kwp_if.h,definition in kwp_if.c, so can use directly with including kwp_if.h */

//hfcpReq_t * delete_ip_addr;
/******************************************************************************
* Function name         : Socket_To_Tx
*    returns            : none
*    parameters         : mode
* Description           : Gets the Connection received Socket Number.
*******************************************************************************/
void Socket_To_Tx(uint8_t sockNum)
{
	hfcpTxSock = sockNum;
}

/******************************************************************************
* Function name     : HFCPRx_Task
* returns           : None
* parameters        : None
* Description       : Handles WiFi Operations
*******************************************************************************/
uint32_t HFCPTxCount = 0;
uint32_t HFCPQueueReceiveCount = 0;
uint32_t hfcpTaskTxLimit = 0;
uint32_t hfcpMaxTxPerSch = 0;
uint32_t ZeroCount;
uint32_t TwoCount;
uint32_t HFCPTxQueueDataPending;

#if HFCP_DEBUG
uint32_t hfcpTaskStart = 0;
uint32_t hfcpTaskStop = 0;
uint32_t hfcpTaskDiffTime = 0;

uint32_t HFCPTaskTickCount = 0;
#endif

uint32_t PrevHFCPTxQueueDataPending = 2;
uint32_t lesscount = 0;
uint32_t HFCPInitCount = 0;
static uint32_t ps_baud_rate = 500000;

#if 0

/*******************************************************************************
* Function Name  : Get_Connect_Flags
* Description    :
* Input          : None.
* Return         : None.
*******************************************************************************/
uint32_t Get_Connect_Flags(uint8_t protocol_id)
{
	if ((protocol_id == CAN_PROTOCOL_ID)
	    || (protocol_id == ISO15765_PROTO_ID)) {
		return CAN_connect_flags;
	}
	if ((protocol_id == CAN_CH1_PROTO_ID)
	    || (protocol_id == ISO15765_CH1_PROTO_ID)) {
		return CAN_CH1_connect_flags;
	} else if (protocol_id == KWP_PROTOCOL_ID) {
		return KWP_connect_flags;
	} else {
		return UNINITIALIZED_PROTO_ID;
	}
}
#endif

void HFCP_setApplicationMode(GARUDA_CommunicationMode_t mode)
{
	if (mode == GARUDA_IN_USB_MODE) {
		DBG("Garuda in USB Mode.");
		HFCPComMode = HFCP_COM_MODE_USB;
	} else if (mode == GARUDA_IN_WIFI_MODE) {
		DBG("Garuda in WiFi Mode.");
		HFCPComMode = HFCP_COM_MODE_WIFI;
	} else {
		HFCPComMode = HFCP_COM_MODE_DEFAULT;
		DBG("Garuda in Default (ie. USB) Mode.");
	}
}

void process_Setup_command(uint8_t * buf)
{
	uint32_t BatteryVg = 0;
	uint8_t *buffer;
	hfcpResp_t *pResp;
	hfcpReq_t *pReq = (hfcpReq_t *) buf;
	int i, status;
	int cmd_len;
	int byte,byte_index=0;

	DBG("Setup command %d", pReq->command);
	cmd_len = sizeof(hfcpReq_t) - sizeof(pReq->u);
	pResp = (hfcpResp_t *) ((uint8_t *) buf + cmd_len);

	memset(pResp, 0, sizeof(hfcpResp_t));
	switch (pReq->command) {
	case Setup_FW_Version:
		DBG("Setup_FW_Version");
		pResp->fw_ver.status = STATUS_NOERROR;
		pResp->fw_ver.buf[0] = 3;
		pResp->fw_ver.buf[1] = 0;
		pResp->fw_ver.buf[2] = 3;
		pResp->fw_ver.buf[3] = 'R';
		/* TODO: In the originial implementation 8 bytes are sent */
		(void)host_write((void *)pReq,
				   cmd_len + sizeof(pResp->fw_ver));
		break;
	case GET_SERIAL_NO:
		DBG("GET_SERIAL_NO");

		if (!G3_board_info_valid) {
			DBG("Failed to read the board info on startup!");
			return EXIT_FAILURE;
		}

		pResp->serialno.status = STATUS_NOERROR;	/** TODO: are we supposed to return success when there is a file error */

		memcpy(pResp->serialno.buf, G3BI.sno, G3_S_SERIAL_NO);
		
		/*Added null character at the end of serial number*/
		//pResp->serialno.buf[G3_S_SERIAL_NO - 1] = '\0'; //NULL character to end string
 
		(void)host_write((void *)pReq,
                                   cmd_len + sizeof(pResp->serialno)+G3_S_SERIAL_NO);
		break;
	case IOCTL_COMMAND:
		DBG("IOCTL_COMMAND");
		if (pReq->u.read_vbatt == Read_VBATT) {
			pResp->vbatt.status = STATUS_NOERROR;
			pResp->vbatt.len = 4;	/* Length */
			if (TRUE) {
				/* TODO: BUG: 'BatteryVg' is used uninitialized */
				pResp->vbatt.BatteryVg = BatteryVg;
			} else {
				pResp->vbatt.status = ERR_FAILED;
				pResp->vbatt.len = 0;
				pResp->vbatt.BatteryVg = 0;
			}
			(void)host_write((void *)pReq,
					   cmd_len + sizeof(pResp->vbatt));
		} else {
			/* Unhandled IOCTL command */
			memset((void *)pReq, 0, cmd_len + sizeof(pResp->vbatt));
			(void)host_write((void *)pReq,
					   cmd_len + sizeof(pResp->vbatt));
		}
		break;
	case Start_Data_logging:
		DBG("Start_Data_logging");
		buffer = (uint8_t *) pReq;
		if ( /*buffer[12] == 0 */ 1) {
			for (i = 0; i < 9; i++) {
				// system_time_stamp[i] = buffer[3+i];
			}
			/* Possible values Datalog buffer size(buff_size_in_MB) are 32MB, 
			 * 64MB, 128MB, 256MB, 512MB, 1024MB, SD_CARD_SIZE_MAX.
			 * !!!Assumed that PC application does the validation of this value!!!
			 */
			buff_size_in_MB =
			    (uint16_t) (buffer[13] | (buffer[14] << 8));

			/* delete the following line once PC app has the Buffer size selection implemented */
			if (buffer[12] == 0) {
				//gl_SDcard_full = 0; 
			}
#if 0
			if (AnyCardIsConnected()) {
				Start_DataLogging_Timestamp();
				Set_DatLog_Event();
			} else {
				FILE_GB_FULL_Led_ON();
			}
#endif
			status = 0;
			if (status) {
				pResp->status = status;
			} else {
				pResp->status = STATUS_NOERROR;
			}
			(void)host_write((void *)pReq, 3);
		}
		break;
	case Stop_Data_logging:
		DBG("Stop_Data_logging");
		buffer = (uint8_t *) pReq;
		if (buffer[2] == 0) {
			if (get_CAN_status() == 1) {
				hfcp_disable_CAN();
			}
			if (get_CAN_CH1_status() == 1) {
				hfcp_disable_CAN_CH1();
			}
			//Clear_DatLog_Event();       
			//Flush_Datalog();
			//Stop_DataLogging_Timestamp();
		} else if (buffer[2] == 1) {
			/* Nothing */
		} else if (buffer[2] == 2) {
			// Clear_DatLog_Event();
		}
		break;
	case Data_logging_mode_query:
		DBG("Data_logging_mode_query");
		(void)host_write((void *)pReq, 3);
		break;
	default:
		DBG("ERR_NOT_SUPPORTED");
		pResp->status = ERR_NOT_SUPPORTED;
		(void)host_write((void *)pReq, 6);
	}
	return;
}

void process_session_mgmt_command(uint8_t * buf)
{
	uint8_t command;
	hfcpResp_t *resp;
	hfcpReq_t *pRxPkt = (hfcpReq_t *) buf;

//      resp = (hfcpResp_t *)((uint8_t *)buf + (sizeof(hfcpReq_t) - sizeof(pRxPkt->u)));
//      resp->status = STATUS_NOERROR;

	command = pRxPkt->command;	/* J2534 Periodic messages */
	DBG("command 0x%x", command);
	DBG("protoid 0x%x", pRxPkt->proto_id);
	DBG("start 0x%x", pRxPkt->u.startsession.startsession);
	DBG("stop 0x%x", pRxPkt->u.stopsession.stopsession);
	switch (command) {
	case Manage_Session:
		if (pRxPkt->u.startsession.startsession == SESSION_OPEN) {
			/* OLD IMPLEMENTATION : start_and_sync_time_stamp(0); */
			// start_time_stamp();
			DBG("Manage_Session - SESSION_OPEN");
			//led_set(LED_USB_LED, LED_ON);
#if GARUDA_USB_DEBUG
			 	
			memset(&Garuda_Debug, 0, sizeof(Garuda_Debug_Info));
			//memset(&Garuda_Debug_CAN2, 0, sizeof(Garuda_Debug_Info));
#endif
			if (checkopendevice) {
				//clear the doip server details
				 doip_cli_delete_server();
			}
			pRxPkt->u.startsession.status = STATUS_NOERROR;
		        
			if (get_connected_channels()) {
				DisconnectAllChanels();
			}
			checkopendevice = 1;
			(void)host_write((void *)pRxPkt, 4);
		} else if (pRxPkt->u.stopsession.stopsession == SESSION_CLOSE) {
			DBG("Manage_Session - SESSION_CLOSE");
			//led_set(LED_USB_LED, LED_OFF);
			checkopendevice = 0;
			if (get_connected_channels()) {
				DisconnectAllChanels();
			}
#if GARUDA_USB_DEBUG
			Garuda_Debug.CAN_Tx_Progress = getCAN1TXStatus();
			//memcpy(&fl_USB_tx_data_U8A[20], &Garuda_Debug, sizeof(Garuda_Debug_Info));
#endif
			pRxPkt->u.stopsession.status = STATUS_NOERROR;

			led_set(LED_USB_LED, LED_OFF);
			led_set(LED_WLANBT_LED, LED_OFF);
			//(void)host_write((void *)pRxPkt, 4);
		} else {
			DBG("Manage_Session - ERR");
			resp =
			    (hfcpResp_t *) ((uint8_t *) buf +
					    (sizeof(hfcpReq_t) -
					     sizeof(pRxPkt->u)));
			resp->status = ERR_NOT_SUPPORTED;
			(void)host_write((void *)pRxPkt, 4);
		}
		break;
	case DATA_LOG_MODE:
#if AMIT
		if (pRxPkt->u.log_mode == DATA_LOG_START) {
			//IMPORTANT start_and_sync_time_stamp(0);
			DBG("DATA_LOG_MODE, log_mode=DATA_LOG_START");
			Set_DatLog_Event();
			(void)host_write((void *)pRxPkt, 4);
		} else if (pRxPkt->u.log_mode == DATA_LOG_STOP) {
			DBG("DATA_LOG_MODE, log_mode=DATA_LOG_STOP");
			/* Stop Timestamp Timer */
			//IMPORTANT disable_timer(3);
			Clear_DatLog_Event();
			Flush_Datalog();
			(void)host_write((void *)pRxPkt, 4);
		} else {
			DBG("DATA_LOG_MODE, NOT_SUPPORTED");
			resp->status = ERR_NOT_SUPPORTED;
			(void)host_write((void *)pRxPkt, 4);
		}
#endif
		break;
	default:
		DBG("ERR_NOT_SUPPORTED");
		resp =
		    (hfcpResp_t *) ((uint8_t *) buf +
				    (sizeof(hfcpReq_t) - sizeof(pRxPkt->u)));
		resp->status = ERR_NOT_SUPPORTED;
		(void)host_write((void *)pRxPkt, 4);
	}
	return;
}

/*******************************************************************************
* Function Name  : process_CAN_command
* Description    :
* Input          : None.
* Return         : None.
*******************************************************************************/
void process_CAN_command(uint8_t * buffer)
{
	uint8_t fl_status_U8 = STATUS_NOERROR;
	struct can_frame frame;
	uint8_t fl_channel_no_U8;
	uint16_t msg_cnt = 0, data_len = 0;
	uint32_t fl_baud_rate_U32, fl_Protocol_ID_U8 = 0;
	uint32_t current_time_stamp = 0;
	uint8_t fl_connection_flag_U8;
	uint8_t base_indx = 0, loop_count = 0;
	uint32_t fl_msg_id = 0;

	// @TODO: set but not used variable, and then unused variable
	// uint32_t fl_tx_flags;
	uint8_t ch_id = 0;

	/* J2534 Periodic messages */
	PERIODIC_MSG j2534_periodic_msg;
	uint8_t periodic_msg_cmd = 0;
	J2534_filter_t fl_filter_config;
	uint8_t fl_FilterID;
	J2534_stError_t fl_filt_config_status, fl_filt_stop_status;
	uint8_t fl_Q_Status = 0;
	uint8_t Ack_Required = 0;
	hfcpReq_t *hfcpreq;
	uint32_t proto_id, i;
	lv_t *plv;
	void *pbuf;
	hfcpResp_t *pRsp;
	int hfcp_proto_cmd_len;

	hfcpreq = (hfcpReq_t *) buffer;

	DBG("CAN command 0x%x\n", hfcpreq->command);
	pbuf = malloc(USB_PKT_SIZE);
	if (pbuf == NULL) {
		DBG("malloc failed");
		return;
	}
	memset(pbuf, 0, sizeof(USB_PKT_SIZE));
	proto_id = hfcpreq->proto_id;
	hfcp_proto_cmd_len = sizeof(hfcpReq_t) - sizeof(hfcpreq->u);
	((hfcpReq_t *) pbuf)->proto_id = proto_id;
	pRsp = (hfcpResp_t *) ((uint8_t *) pbuf + hfcp_proto_cmd_len);

	switch (hfcpreq->command) {
	case CAN_EnableComm:
		fl_status_U8 = STATUS_NOERROR;
		fl_baud_rate_U32 = hfcpreq->u.can_enable.baudrate;
		/* Handle CAN Channel 1 */
		if ((proto_id == CAN_PROTOCOL_ID) ||
		    (proto_id == ISO15765_PROTO_ID)) {
			CAN1_Vehicle_STMinValue = 0;
			fl_channel_no_U8 = CAN_CH1;
			//strcpy(fl_channel_no_U8,CAN_CH1);
			CAN_or_ISO = proto_id;
			CAN_or_ISO15765_J1939 = proto_id;
			CAN_connect_flags = hfcpreq->u.can_enable.conn_flags;
			DBG("CAN 1 Enable Communication Cmd");
			fl_status_U8 = CAN_Mid_Init(CAN_CHA1, fl_baud_rate_U32);
			//CAN_Mid_Init (char *can, uint32_t p_baudrate);
			if ((ERR_INVALID_BAUDRATE != fl_status_U8)
			    && (ERR_INVALID_CHANNEL_ID != fl_status_U8)) {
				DBG("Check success");
				if ((CAN_connect_flags & CAN_ID_BOTH) ==
				    CAN_ID_BOTH) {
					fl_connection_flag_U8 =
					    RX_BOTH_STD_EXT_MSG;
				} else if ((CAN_connect_flags & CAN_29BIT_ID)
					   == CAN_29BIT_ID) {
					fl_connection_flag_U8 = RX_ONLY_EXT_MSG;
				} else if ((CAN_connect_flags & CAN_29BIT_ID)
					   == 0) {
					fl_connection_flag_U8 = RX_ONLY_STD_MSG;
				} else {
					fl_status_U8 = ERR_INVALID_FLAGS;
				}
				if (fl_status_U8 != ERR_INVALID_FLAGS) {
					fl_status_U8 =
					    CAN_Mid_Enable(CAN_CH1,
							   fl_connection_flag_U8);
					//CAN_Mid_Init (char *can, uint32_t p_baudrate);
					if (fl_status_U8 == STATUS_NOERROR) {
						l_connected_channels++;
						l_connected_channel_1 = 1;
					}
				} else {
					/* Do nothing as fl_status_U8 has been modified */
				}
			} else {
				/* Do nothing as fl_status_U8 has been modified */
			}
			if (fl_status_U8 == STATUS_NOERROR) {
				CAN_active = 1;
				led_set(LED_LINK_LED, LED_ON);
			}
			((hfcpReq_t *) pbuf)->command = CAN_EnableComm_ACK;
			pRsp->status = fl_status_U8;
			(void)host_write((void *)pbuf,
					   hfcp_proto_cmd_len +
					   sizeof(pRsp->status));
		}
		/* Handle CAN Channel 2 */
		else if ((proto_id == CAN_CH1_PROTO_ID) ||
			 (proto_id == ISO15765_CH1_PROTO_ID)) {
			CAN2_vehicle_STMinValue = 0;
			fl_channel_no_U8 = CAN_CH2;
			//strcpy(fl_channel_no_U8,CAN_CHA2);
			CAN1_or_ISO1 = proto_id;
			CANCH1_or_ISO15765CH1_J1939CH1 = proto_id;
			CAN_CH1_connect_flags = hfcpreq->u.can_enable.conn_flags;
			DBG("CAN 2 Enable Communication Cmd \r\n");
			fl_status_U8 = CAN_Mid_Init(CAN_CHA2, fl_baud_rate_U32);
			if ((ERR_INVALID_BAUDRATE != fl_status_U8)
			    && (ERR_INVALID_CHANNEL_ID != fl_status_U8)) {
				if ((CAN_CH1_connect_flags & CAN_ID_BOTH)
				    == CAN_ID_BOTH) {
					fl_connection_flag_U8 =
					    RX_BOTH_STD_EXT_MSG;
				} else
				    if ((CAN_CH1_connect_flags &
					 CAN_29BIT_ID) == CAN_29BIT_ID) {
					fl_connection_flag_U8 = RX_ONLY_EXT_MSG;
				} else
				    if ((CAN_CH1_connect_flags &
					 CAN_29BIT_ID) == 0) {
					fl_connection_flag_U8 = RX_ONLY_STD_MSG;
				} else {
					fl_status_U8 = ERR_INVALID_FLAGS;
				}
				if (fl_status_U8 != ERR_INVALID_FLAGS) {
					fl_status_U8 =
					    CAN_Mid_Enable(CAN_CH2,
							   fl_connection_flag_U8);
					if (fl_status_U8 == STATUS_NOERROR) {
						l_connected_channels++;
						l_connected_channel_2 = 1;
					}
				} else {
					/* Do nothing as fl_status_U8 has been modified */
				}
			} else {
				/* Do nothing as fl_status_U8 has been modified */
			}
			if (fl_status_U8 == STATUS_NOERROR) {
				CAN_CH1_active = 1;
				led_set(LED_LINK_LED, LED_ON);
			}
			((hfcpReq_t *) pbuf)->command = CAN_EnableComm_ACK;
			pRsp->status = fl_status_U8;
			(void)host_write((void *)pbuf,
					   hfcp_proto_cmd_len +
					   sizeof(pRsp->status));
		} else {
			/* ERROR: It should never enter here */
			pRsp->status = ERR_INVALID_PROTOCOL_ID;
			(void)host_write((void *)pbuf,
					   hfcp_proto_cmd_len +
					   sizeof(pRsp->status));
		}
		break;
	case CAN_DisableComm:
		if ((proto_id == CAN_PROTOCOL_ID) ||
		    (proto_id == ISO15765_PROTO_ID)) {
			fl_channel_no_U8 = CAN_CH1;
			//strcpy(fl_channel_no_U8,CAN_CHA1);
			CAN_or_ISO = 0;
			CAN_or_ISO15765_J1939 = 0;
			//STMinValue =0;
			CAN1_Vehicle_STMinValue = 0;
			/* Clears All the Filters Configured */
			J2534_ClearAllFilter(CAN_PROTOCOL_ID);
			/* Suspend all periodic messages */
			//suspend_pmsg(CAN_PROTOCOL_ID);
			suspend_pmsg(CAN_PROTOCOL_ID);
			//CAN_ClearQ(CAN1_TX_QUEUE);
			if (getCAN1TXStatus()) {
				//xSemaphoreGive(xCAN1StartTx);
			}
			//CAN_ClearQ(CAN1_RX_QUEUE);
#if GARUDA_DEBUG
			DBG("HFCP DEBUG : CAN Disable Communication Cmd");
#endif
			// fl_status_U8 = CAN_Mid_Disable(fl_channel_no_U8);
			if (fl_status_U8 == STATUS_NOERROR) {
				CAN_active = 0;
				led_set(LED_LINK_LED, LED_OFF);
				if (l_connected_channels) {
					l_connected_channels--;
				}
				l_connected_channel_1 = 0;
			}
			CAN_connect_flags = 0;
			((hfcpReq_t *) pbuf)->command = CAN_DisableComm_ACK;
			pRsp->status = fl_status_U8;
			(void)host_write((void *)pbuf,
					   hfcp_proto_cmd_len +
					   sizeof(pRsp->status));

			/* Added the Debug Messages in USB Frame for ACK Packet */
#if GARUDA_USB_DEBUG
			// Garuda_Debug.CAN_Tx_Progress = getCAN1TXStatus();
			// memcpy(&fl_USB_tx_data_U8A[20], &Garuda_Debug, sizeof(Garuda_Debug_Info));
#endif

			/* Garuda_init(); */
			/* Commented to fix the issue of not transmitting Ack for DisableComm
			 * Side effects are yet to be investigated
			 */
		} else if ((proto_id == CAN_CH1_PROTO_ID) ||
			   (proto_id == ISO15765_CH1_PROTO_ID)) {
			fl_channel_no_U8 = CAN_CH2;
			//strcpy(fl_channel_no_U8,CAN_CHA2);
			CAN1_or_ISO1 = 0;
			CANCH1_or_ISO15765CH1_J1939CH1 = 0;
			CAN2_vehicle_STMinValue = 0;
			/* Clears All the Filters Configured */
			J2534_ClearAllFilter(CAN_CH1_PROTO_ID);
			/* Suspend all periodic messages */
			// suspend_pmsg(CAN_CH1_PROTO_ID); 
			suspend_pmsg(CAN_CH1_PROTO_ID); 

			//CAN_ClearQ(CAN2_TX_QUEUE);
#if 0
			if (getCAN2TXStatus()) {
				xSemaphoreGive(xCAN2StartTx);
			}
#endif
			//CAN_ClearQ(CAN2_RX_QUEUE);
			//fl_status_U8 = CAN_Mid_Disable(fl_channel_no_U8);
			if (fl_status_U8 == STATUS_NOERROR) {
				CAN_CH1_active = 0;
				led_set(LED_LINK_LED, LED_OFF);
				if (l_connected_channels) {
					l_connected_channels--;
				}
				l_connected_channel_2 = 0;
			}
			CAN_CH1_connect_flags = 0;
			((hfcpReq_t *) pbuf)->command = CAN_DisableComm_ACK;
			pRsp->status = fl_status_U8;
			(void)host_write((void *)pbuf,
					   hfcp_proto_cmd_len +
					   sizeof(pRsp->status));
			/*
			 * #if GARUDA_USB_DEBUG
			 * Garuda_Debug_CAN2.CAN_Tx_Progress = getCAN2TXStatus();
			 * memcpy(&fl_USB_tx_data_U8A[20], &Garuda_Debug_CAN2, sizeof(Garuda_Debug_Info));
			 * #endif
			 */
			/* Garuda_init(); *//* Commented to fix the issue of not transmitting Ack for DisableComm
			   Side effects are yet to be investigated */
		} else {
			/* ERROR: It should never enter here */
			((hfcpReq_t *) pbuf)->command = CAN_DisableComm_ACK;
			pRsp->status = ERR_INVALID_PROTOCOL_ID;
			(void)host_write((void *)pbuf,
					   hfcp_proto_cmd_len +
					   sizeof(pRsp->status));
		}
		break;
	case CAN_IOCTL_COMMAND:
		process_CAN_IOCTL_cmd(buffer);
		break;
	case CAN_Send_msg:
		switch (proto_id) {
		case CAN_PROTOCOL_ID:
		case ISO15765_PROTO_ID:
			ch_id = 1;
			break;
		case CAN_CH1_PROTO_ID:
		case ISO15765_CH1_PROTO_ID:
			ch_id = 2;
			break;
		default:
			/* Error */
		}
		HFCPInitCount++;
		if (hfcpreq->u.can_sndmsg.seg_num == 0) {
			data_len = hfcpreq->u.can_sndmsg.dlen;

			DBG("hfcpreq->u.can_sndmsg.dlen=%x",
			    hfcpreq->u.can_sndmsg.dlen);
			DBG("Deciding data length in process_CAN_Command=%d %x",
			    data_len, data_len);
			DBG("data_len & CAN_DATA_LEN_MASK = %x",
			    (data_len & CAN_DATA_LEN_MASK));

			memset((void *)&frame, 0, sizeof(frame));
			if ((data_len & CAN_DATA_LEN_MASK) == 0x0000) {
				/* Mode 0 */
				DBG("Inside case 0x0000. Mode one");
				if (ch_id == 1) {
					CAN1_Vehicle_STMinValue = 0;
				} else {
					CAN2_vehicle_STMinValue = 0;
				}
				data_len &= 0x3FFF;
				DBG("data_len &= 0x3FFF=%x", data_len);
				frame.can_id =
				    CANID_FROM_DATA(&
						    (hfcpreq->u.can_sndmsg.
						     canid));
				frame.can_dlc = data_len - 4;
				memcpy(&frame.data[0],
				       hfcpreq->u.can_sndmsg.buf,
				       CAN_DL_MIN_SIZE(frame.can_dlc,
						       sizeof(frame.data)));
				if (CAN_EXT_MSG ==
				    (hfcpreq->u.can_sndmsg.
				     tx_flags & CAN_EXT_MSG)) {
					/* Extended Frame */
					frame.can_id &= CAN_EFF_MASK;
					frame.can_id |= CAN_EFF_FLAG;
				} else {
					/* Standard Frame */
					frame.can_id &= CAN_SFF_MASK;
				}
				if (ch_id == 1) {
					fl_Q_Status =
					    CAN_Mid_Transmit(CAN_CH1, &frame);
				} else if (ch_id == 2) {
					fl_Q_Status =
					    CAN_Mid_Transmit(CAN_CH2, &frame);
				}
				/* TODO: fl_Q_Status is used unitialized */
				if (fl_Q_Status == MID_FAIL) {
					/* Message lost as the Buffer is full */
					((hfcpReq_t *) pbuf)->command =
					    CAN_Send_msg_ACK;
					pRsp->sndmsg_rsp.seg_num = hfcpreq->u.can_sndmsg.seg_num;	/* Segment number */
					pRsp->sndmsg_rsp.status =
					    ERR_BUFFER_FULL;
					(void)host_write((void *)pbuf,
							   hfcp_proto_cmd_len
							   +
							   sizeof
							   (pRsp->sndmsg_rsp));
				} else {
					((hfcpReq_t *) pbuf)->command =
					    CAN_Send_msg_ACK;
					pRsp->sndmsg_rsp.seg_num = hfcpreq->u.can_sndmsg.seg_num;	/* Segment number */
					pRsp->sndmsg_rsp.status = fl_status_U8;
					pRsp->sndmsg_rsp.timestamp =
					    current_time_stamp;
					(void)host_write((void *)pbuf,
							   hfcp_proto_cmd_len
							   +
							   sizeof
							   (pRsp->sndmsg_rsp));
				}
			} else if ((data_len & CAN_DATA_LEN_MASK) == 0x4000) {
				/* Mode 1 */
				if (ch_id == 1) {
					CAN1_Vehicle_STMinValue = 0;
				} else {
					CAN2_vehicle_STMinValue = 0;
				}
				/* buffer[5] - Supposed to hold
				   conversation Id - To be incorporated */
				stored_tx_flags_CAN =
				    hfcpreq->u.can_sndmsg_m1.tx_flags;
				if (ch_id == 1) {
					stored_msg_id_CAN =
					    CANID_FROM_DATA(&
							    (hfcpreq->u.
							     can_sndmsg_m1.
							     canid));
					frame.can_id = stored_msg_id_CAN;
				} else if (ch_id == 2) {
					stored_msg_id_CAN_CH1 =
					    CANID_FROM_DATA(&
							    (hfcpreq->u.
							     can_sndmsg_m1.
							     canid));
					frame.can_id = stored_msg_id_CAN_CH1;
				} else {
					/* Error */
				}
				data_len &= 0x3FFF;
				frame.can_dlc = data_len - 4;
				memcpy(&frame.data[0],
				       hfcpreq->u.can_sndmsg_m1.buf,
				       CAN_DL_MIN_SIZE(frame.can_dlc,
						       sizeof(frame.data)));
				if (CAN_EXT_MSG ==
				    (stored_tx_flags_CAN & CAN_EXT_MSG)) {
					/* Extended Frame */
					frame.can_id &= CAN_EFF_MASK;
					frame.can_id |= CAN_EFF_FLAG;
				} else {
					/* Standard Frame */
					frame.can_id &= CAN_SFF_MASK;
				}
				DBG("Mode 1: msg_id=0x%x", frame.can_id);
				if (ch_id == 1) {
					fl_Q_Status =
					    CAN_Mid_Transmit(CAN_CH1, &frame);
				} else if (ch_id == 2) {
					fl_Q_Status =
					    CAN_Mid_Transmit(CAN_CH2, &frame);
				}
				if (fl_Q_Status == MID_FAIL) {
					((hfcpReq_t *) pbuf)->command =
					    CAN_Send_msg_ACK;
					pRsp->sndmsg_rsp.seg_num = hfcpreq->u.can_sndmsg_m1.seg_num;	/* Segment number */
					pRsp->sndmsg_rsp.status =
					    ERR_BUFFER_FULL;
					(void)host_write((void *)pbuf,
							   hfcp_proto_cmd_len
							   +
							   sizeof
							   (pRsp->sndmsg_rsp));
				} else {
					((hfcpReq_t *) pbuf)->command =
					    CAN_Send_msg_ACK;
					pRsp->sndmsg_rsp.seg_num = hfcpreq->u.can_sndmsg_m1.seg_num;	/* Segment number */
					pRsp->sndmsg_rsp.status = fl_status_U8;
					pRsp->sndmsg_rsp.timestamp =
					    current_time_stamp;
#if GARUDA_USB_DEBUG
					if (ch_id == 1) {
						Garuda_Debug.CAN_Tx_Progress
						    = getCAN1TXStatus();
						memcpy(&fl_USB_tx_data_U8A
						       [20], &Garuda_Debug,
						       sizeof
						       (Garuda_Debug_Info));
					} else {
						Garuda_Debug_CAN2.
						    CAN_Tx_Progress =
						    getCAN2TXStatus();
						memcpy(&fl_USB_tx_data_U8A[20],
						       &Garuda_Debug_CAN2,
						       sizeof
						       (Garuda_Debug_Info));
					}
#endif
					(void)host_write((void *)pbuf,
							   hfcp_proto_cmd_len
							   +
							   sizeof
							   (pRsp->sndmsg_rsp));
				}
			} else if ((data_len & CAN_DATA_LEN_MASK) == 0x8000) {
				/* Mode 2 */
				/* buffer[5] - Supposed to hold
				 * conversation Id - To be incorporated
				 * buffer[6],[7] - Supposed to hold
				 * Separation time - To be incorporated  */
				memset((void *)&frame, 0, sizeof(frame));
				if (ch_id == 1) {
					CAN1_Vehicle_STMinValue =
					    hfcpreq->u.can_sndmsg_m2.STMinValue;
				} else {
					CAN2_vehicle_STMinValue =
					    hfcpreq->u.can_sndmsg_m2.STMinValue;
				}
#if 0
				if (HFCPComMode == HFCP_COM_MODE_USB) {
					Ack_Required = 1;
				} else if (HFCPComMode == HFCP_COM_MODE_WIFI) {
					Ack_Required = 1;
					   //hfcpreq->u.can_sndmsg_m2.ack_req;
				}
#endif
				Ack_Required = 1;

				if (ch_id == 1) {
					frame.can_id = stored_msg_id_CAN;
				} else {
					frame.can_id = stored_msg_id_CAN_CH1;
				}
				if (CAN_EXT_MSG ==
				    (stored_tx_flags_CAN & CAN_EXT_MSG)) {
					/* Extended Frame */
					frame.can_id &= CAN_EFF_MASK;
					frame.can_id |= CAN_EFF_FLAG;
				} else {
					/* Standard Frame */
					frame.can_id &= CAN_SFF_MASK;
				}
				data_len &= 0x3FFF;
				if (data_len <= 501) {
					if ((data_len % 8) != 0) {
						msg_cnt = (data_len / 8) + 1;
					} else {
						msg_cnt = (data_len / 8);
					}
					base_indx = 0;
					while (msg_cnt--) {
						//fl_CAN_Tx_msg_S.rtr = 0;
						/* Copy data length */
						if (data_len >= 8) {
							data_len -= 8;
							frame.can_dlc = 8;
						} else {
							frame.can_dlc =
							    data_len;
							data_len = 0;
						}
						/* copy all data bytes into buffer */
						memcpy(&frame.data[0],
						       &hfcpreq->u.
						       can_sndmsg_m2.
						       buf[base_indx],
						       CAN_DL_MIN_SIZE(frame.
								       can_dlc,
								       sizeof
								       (frame.
									data)));
						if (ch_id == 1) {
							//      fl_Q_Status = CAN_Mid_Transmit(CAN_CH1, &frame);
							if (CAN1_Vehicle_STMinValue) {
								hexdump
								    ("To Q, ch1",
								     &frame,
								     sizeof
								     (frame),
								     0);
								fl_Q_Status
								    =
								    mode2_queue_add_to_queue
								    (CAN_CH1,
								     (char *)&frame,
								     CAN1_Vehicle_STMinValue);
							} else {
								fl_Q_Status
								    =
								    CAN_Mid_Transmit
								    (CAN_CH1,
								     &frame);
							}
						} else if (ch_id == 2) {
							//fl_Q_Status = CAN_Mid_Transmit(CAN_CH2, &frame);
							if (CAN2_vehicle_STMinValue) {
								hexdump
								    ("To Q, ch2",
								     &frame,
								     sizeof
								     (frame),
								     0);
								fl_Q_Status
								    =
								    mode2_queue_add_to_queue
								    (CAN_CH2,
								     (char *)&frame,
								     CAN2_vehicle_STMinValue);
							} else {
								fl_Q_Status
								    =
								    CAN_Mid_Transmit
								    (CAN_CH2,
								     &frame);
							}
						}
						if (MID_FAIL != fl_Q_Status) {
							base_indx += 8;
						} else {
							/* Do nothing */
							/* Note that the "while" in this "else if" is blocking.
							   i.e. till it writes all the frames to CAN interface it will
							   not exit. A queue strategy is to be implemented such
							   that there is always space for 7 frames.
							 */
						}
					}	/* while */
				} else {
					fl_status_U8 = ERR_FAILED;
				}

				/* Clear the USB buffer as it has been processed */
				if (Ack_Required == TRUE) {
					((hfcpReq_t *) pbuf)->command =
					    CAN_Send_msg_ACK;
					pRsp->sndmsg_rsp.seg_num = hfcpreq->u.can_sndmsg_m2.seg_num;	/* Segment number */
					pRsp->sndmsg_rsp.status = fl_status_U8;
					pRsp->sndmsg_rsp.timestamp =
					    current_time_stamp;
					(void)host_write((void *)pbuf,
							   hfcp_proto_cmd_len
							   +
							   sizeof
							   (pRsp->sndmsg_rsp));
				}
			}	/* data_len == 0x8000 */

			/* Multiple Messages in Single USB */
			else if ((data_len & CAN_DATA_LEN_MASK) ==
				 CAN_DATA_LEN_MASK) {
				struct mode_data *p;

				p = (struct mode_data *)hfcpreq->u.
				    can_sndmsg_m3.data;
				/* Mode 3 */
				DBG("Inside C000 case. In Mode Three.");
				//((ch_id == 1) ? (CAN1_Vehicle_STMinValue = 0) : (CAN2_vehicle_STMinValue = 0));       
				base_indx = 5;
				msg_cnt = (uint8_t) (data_len & 0x3FFF);
				DBG("In Mode 3: msg_cnt=%x %d", msg_cnt,
				    msg_cnt);
				while (msg_cnt--) {
					fl_msg_id = CANID_FROM_DATA(&p->canid);
					data_len = p->dlen;
					memset((void *)&frame, 0,
					       sizeof(frame));
					frame.can_id = fl_msg_id;
					frame.can_dlc = data_len;
					memcpy(&frame.data[0], p->buf,
					       CAN_DL_MIN_SIZE
					       (frame.can_dlc,
						sizeof(frame.data)));
					if (CAN_EXT_MSG ==
					    (hfcpreq->u.can_sndmsg_m3.tx_flags &
					     CAN_EXT_MSG)) {
						/* Extended Frame */
						frame.can_id &= CAN_EFF_MASK;
						frame.can_id |= CAN_EFF_FLAG;
					} else {
						/* Standard Frame */
						frame.can_id &= CAN_SFF_MASK;
					}
					do {
						if (ch_id == 1) {
							fl_Q_Status =
							    CAN_Mid_Transmit
							    (CAN_CH1, &frame);
						} else if (ch_id == 2) {
							fl_Q_Status =
							    CAN_Mid_Transmit
							    (CAN_CH2, &frame);
						}
					} while (fl_Q_Status ==
						 ERR_BUFFER_FULL);

					base_indx += data_len;
					p = (struct mode_data
					     *)((uint8_t *) p + data_len);
				}	// while()
				((hfcpReq_t *) pbuf)->command =
				    CAN_Send_msg_ACK;
				pRsp->sndmsg_rsp.seg_num =
				    hfcpreq->u.can_sndmsg_m3.seg_num;
				pRsp->sndmsg_rsp.status = fl_status_U8;
				pRsp->sndmsg_rsp.timestamp = current_time_stamp;
				(void)host_write((void *)pbuf,
						   hfcp_proto_cmd_len +
						   sizeof(pRsp->sndmsg_rsp));
			}
			/* ERROR : Illegal Mode */
			else {
				((hfcpReq_t *) pbuf)->command =
				    CAN_Send_msg_ACK;
				pRsp->sndmsg_rsp.seg_num = hfcpreq->u.can_sndmsg_m3.seg_num;	/* Segment number */
				pRsp->sndmsg_rsp.status = ERR_FAILED;
				(void)host_write((void *)pbuf,
						   hfcp_proto_cmd_len +
						   sizeof(pRsp->sndmsg_rsp));
			}
		}		// buffer[2] == 0

		/* ERROR:For CAN segmented USB transfer is not needed */
		else {
			((hfcpReq_t *) pbuf)->command = CAN_Send_msg_ACK;
			pRsp->sndmsg_rsp.seg_num = hfcpreq->u.can_sndmsg_m3.seg_num;	/* Segment number */
			pRsp->sndmsg_rsp.status = ERR_FAILED;
			(void)host_write((void *)pbuf,
					   hfcp_proto_cmd_len +
					   sizeof(pRsp->sndmsg_rsp));
		}
		break;
	case Start_msg_filter:
		if ((proto_id == ISO15765_PROTO_ID) ||
		    (proto_id == CAN_PROTOCOL_ID)) {
			fl_Protocol_ID_U8 = CAN_PROTOCOL_ID;

		} else if ((proto_id == ISO15765_CH1_PROTO_ID) ||
			   (proto_id == CAN_CH1_PROTO_ID)) {
			fl_Protocol_ID_U8 = CAN_CH1_PROTO_ID;
		}
		/* TODO: fl_Protocol_ID_U8 is used uninitilized */
		fl_filter_config.Protocol_ID = fl_Protocol_ID_U8;
		fl_filter_config.filterType =
		    (J2534_filterType_t) (hfcpreq->u.can_msg_filter.filtertype);
		plv = (lv_t *) (hfcpreq->u.can_msg_filter.buf);
		fl_filter_config.MaskLen = plv->len;
		for (i = 0; i < plv->len; i++) {
			fl_filter_config.maskMsg[i] = plv->buf[i];
		}
		plv =
		    (lv_t *) ((hfcpreq->u.can_msg_filter.buf) +
			      sizeof(plv->len) + plv->len);
		fl_filter_config.PatternLen = plv->len;
		for (i = 0; i < plv->len; i++) {
			fl_filter_config.patternMsg[i] = plv->buf[i];
		}
		fl_filt_config_status =
		    J2534_ConfigFilter(&fl_filter_config, &fl_FilterID);

		((hfcpReq_t *) pbuf)->command = Start_msg_filter_ACK;
		pRsp->msg_fil.filterid = fl_FilterID;

		if (fl_filt_config_status != J2534_NO_ERROR) {
			if (fl_filt_config_status == J2534_FLT_NOT_FREE) {
				fl_status_U8 = ERR_EXCEEDED_LIMIT;
			} else {
				fl_status_U8 = ERR_FAILED;
			}
			pRsp->msg_fil.status = fl_status_U8;
		} else {
			pRsp->msg_fil.status = STATUS_NOERROR;
		}
		(void)host_write((void *)pbuf,
				   hfcp_proto_cmd_len + sizeof(pRsp->msg_fil));
		break;
	case Stop_msg_filter:
		switch (proto_id) {
		case ISO15765_PROTO_ID:
		case CAN_PROTOCOL_ID:
			fl_Protocol_ID_U8 = CAN_PROTOCOL_ID;
			break;
		case ISO15765_CH1_PROTO_ID:
		case CAN_CH1_PROTO_ID:
			fl_Protocol_ID_U8 = CAN_CH1_PROTO_ID;
			break;
		default:
			DBG("Invalid proto_id");
			/* error */
		}
		fl_FilterID = hfcpreq->u.can_stpmsg_filter.filterid;
		fl_filt_stop_status =
		    J2534_ClearFilter(fl_FilterID, fl_Protocol_ID_U8);

		((hfcpReq_t *) pbuf)->command = Stop_msg_filter_ACK;
		pRsp->stpmsg_fil.filterid = fl_FilterID;
		if (fl_filt_stop_status != J2534_NO_ERROR) {
			if (fl_filt_stop_status == J2534_INVLD_FLTID) {
				fl_status_U8 = ERR_INVALID_FILTER_ID;
			} else {
				fl_status_U8 = ERR_FAILED;
			}
			((hfcpReq_t *) pbuf)->proto_id = fl_Protocol_ID_U8;
			pRsp->stpmsg_fil.status = fl_status_U8;
		} else {
			pRsp->msg_fil.status = STATUS_NOERROR;
		}
		(void)host_write((void *)pbuf,
				   hfcp_proto_cmd_len +
				   sizeof(pRsp->stpmsg_fil));
		DBG("HFCP DEBUG : Stop Mesage Filter = %d \r\n", fl_status_U8);
		break;
	case handle_periodic_msg:
		switch (proto_id) {
		case ISO15765_PROTO_ID:
		case CAN_PROTOCOL_ID:
			fl_Protocol_ID_U8 = CAN_PROTOCOL_ID;
			break;
		case ISO15765_CH1_PROTO_ID:
		case CAN_CH1_PROTO_ID:
			fl_Protocol_ID_U8 = CAN_CH1_PROTO_ID;
			break;
		default:
			DBG("Invalid proto_id");
			/* error */
		}
		periodic_msg_cmd = hfcpreq->u.can_periodic_msg.periodic_command;
		j2534_periodic_msg.protocol_id = fl_Protocol_ID_U8;
		j2534_periodic_msg.periodicity =
		    hfcpreq->u.can_periodic_msg.periodicity;
		j2534_periodic_msg.prmsg_id = hfcpreq->u.can_periodic_msg.msgid;
		j2534_periodic_msg.proto_msg.tx_flags =
		    hfcpreq->u.can_periodic_msg.tx_flags;
		if ((periodic_msg_cmd == START_NEW_PERIODIC_MSG_TXN)
		    || (periodic_msg_cmd == UPDATE_DATA_TO_MSG_ID)) {
			j2534_periodic_msg.proto_msg.length =
			    hfcpreq->u.can_periodic_msg.msglen;
		} else {
			j2534_periodic_msg.proto_msg.length = 0;
		}
		for (loop_count = 0;
		     loop_count < j2534_periodic_msg.proto_msg.length;
		     loop_count++) {
			j2534_periodic_msg.proto_msg.data[loop_count] =
			    hfcpreq->u.can_periodic_msg.buf[loop_count];
		}

		if (PERIODIC_SUCCESS ==
		    PERIODIC_msg_cmd(&j2534_periodic_msg, periodic_msg_cmd)) {
			fl_status_U8 = STATUS_NOERROR;
		} else {
			fl_status_U8 = ERR_FAILED;
		}
		((hfcpReq_t *) pbuf)->command = handle_periodic_msg_ACK;
		pRsp->pmsg.status = fl_status_U8;
		pRsp->pmsg.per_msg_id = j2534_periodic_msg.prmsg_id;
		(void)host_write((void *)pbuf,
				   hfcp_proto_cmd_len + sizeof(pRsp->pmsg));
		break;
	default:
		break;

		free(pbuf);
	}
}

void process_CAN_FD_command(uint8_t * buffer)
{
	uint8_t fl_status_U8 = STATUS_NOERROR;

	/* uint8_t fl_USB_tx_data_U8A[64]; */
	uint8_t fl_channel_no_U8;
	uint8_t fl_connection_flag_U8;
	uint32_t fl_baud_rate_U32;
	uint8_t ch_id = 0, fl_Q_Status = 0;
	uint16_t msg_cnt = 0, data_len = 0;
	struct canfd_frame frame;
	uint32_t fl_msg_id = 0;
	uint32_t current_time_stamp = 0;
	uint8_t Ack_Required = 0;
	uint16_t base_indx = 0, loop_count = 0;
	uint8_t fl_FilterID;
	uint32_t fl_Protocol_ID_U8 = 0;
	J2534_stError_t fl_filt_config_status, fl_filt_stop_status;
	J2534_filter_t fl_filter_config;
	uint8_t periodic_msg_cmd = 0;
	PERIODIC_MSG j2534_periodic_msg;
	hfcpReq_t *hfcpreq;
	uint32_t proto_id, i;
	lv_t *plv;
	void *pbuf;
	int hfcp_proto_cmd_len;
	hfcpResp_t *pRsp;

	hfcpreq = (hfcpReq_t *) buffer;

	DBG("CAN FD command 0x%x\tDATE-02052024\n", hfcpreq->command);
	pbuf = calloc(1, USB_PKT_SIZE);
	if (pbuf == NULL) {
		DBG("PBUF Allocation");
		return;
	}

	/*     memset(pbuf, 0, sizeof(USB_PKT_SIZE));//aready using calloc() */
	proto_id = hfcpreq->proto_id;
	hfcp_proto_cmd_len = sizeof(hfcpReq_t) - sizeof(hfcpreq->u);
	((hfcpReq_t *) pbuf)->proto_id = proto_id;
	pRsp = (hfcpResp_t *) ((uint8_t *) pbuf + hfcp_proto_cmd_len);

	DBG("proto_id=0x%x\n", proto_id);

	switch (hfcpreq->command) {
	case CAN_FD_EnableComm:
		fl_status_U8 = STATUS_NOERROR;
		fl_baud_rate_U32 = hfcpreq->u.can_enable.baudrate;
		/* Handle CAN Channel 1 */
		if ((proto_id == CAN_FD_PROTOCOL_ID) ||
		    (proto_id == ISO15765_FD_PROTO_ID)) {
			CAN1_Vehicle_STMinValue = 0;
			fl_channel_no_U8 = CAN_CH1;

			CAN_or_ISO = proto_id;
			CAN_FD_or_ISO15765_FD = proto_id;
			CAN_connect_flags = hfcpreq->u.can_enable.conn_flags;
			DBG("HFCP DEBUG : CAN_FD CAN1 Enable Communication Cmd\n");
			fl_status_U8 = CAN_Mid_Init(CAN_CHA1, fl_baud_rate_U32);
			if ((ERR_INVALID_BAUDRATE != fl_status_U8)
			    && (ERR_INVALID_CHANNEL_ID != fl_status_U8)) {
				DBG("Check success.\n");
				if ((CAN_connect_flags & CAN_ID_BOTH) ==
				    CAN_ID_BOTH) {
					fl_connection_flag_U8 =
					    RX_BOTH_STD_EXT_MSG;
				} else if ((CAN_connect_flags & CAN_29BIT_ID)
					   == CAN_29BIT_ID) {
					fl_connection_flag_U8 = RX_ONLY_EXT_MSG;
				} else if ((CAN_connect_flags & CAN_29BIT_ID)
					   == 0) {
					fl_connection_flag_U8 = RX_ONLY_STD_MSG;
				} else {
					fl_status_U8 = ERR_INVALID_FLAGS;
				}
				if (fl_status_U8 != ERR_INVALID_FLAGS) {
					fl_status_U8 =
					    CAN_Mid_Enable(CAN_CH1,
							   fl_connection_flag_U8);
					//CAN_Mid_Init (char *can, uint32_t p_baudrate);
					if (fl_status_U8 == STATUS_NOERROR) {
						l_connected_channels++;
						l_connected_channel_1 = 1;
					}
				} else {
					/* Do nothing as fl_status_U8 has been modified */
				}
			} else {
				/* Do nothing as fl_status_U8 has been modified */
			}
			if (fl_status_U8 == STATUS_NOERROR) {

				CAN_active = 1;
				led_set(LED_LINK_LED, LED_ON);
			}
			/*
			   fl_USB_tx_data_U8A[0] = proto_id;
			   fl_USB_tx_data_U8A[1] = CAN_EnableComm_ACK;
			   fl_USB_tx_data_U8A[3] = fl_status_U8;
			   DBG("Before Garuda_Tx_data_on_USB.\t\tfl_USB_tx_data_U8A=%x %x %x %x\n",
			   fl_USB_tx_data_U8A[0], fl_USB_tx_data_U8A[1],
			   fl_USB_tx_data_U8A[2],fl_USB_tx_data_U8A[3]);

			   (void) host_write((void *)fl_USB_tx_data_U8A, 4); */
			((hfcpReq_t *) pbuf)->command = CAN_FD_EnableComm_ACK;
			pRsp->status = fl_status_U8;
			(void)host_write((void *)pbuf,
					   hfcp_proto_cmd_len +
					   sizeof(pRsp->status));

		}
		/* Handle CAN Channel 2 */
		else if ((proto_id == CAN_FD_CH1_PROTO_ID) ||
			 (proto_id == ISO15765_FD_CH1_PROTO_ID)) {
			CAN2_vehicle_STMinValue = 0;
			fl_channel_no_U8 = CAN_CH2;

			CAN1_or_ISO1 = proto_id;
			CAN_FDCH1_or_ISO15765_FDCH1 = proto_id;
			CAN_CH1_connect_flags =
			    hfcpreq->u.can_enable.conn_flags;

			fl_status_U8 = CAN_Mid_Init(CAN_CHA2, fl_baud_rate_U32);

			if ((ERR_INVALID_BAUDRATE != fl_status_U8) &&
			    (ERR_INVALID_CHANNEL_ID != fl_status_U8)) {
				if ((CAN_CH1_connect_flags & CAN_ID_BOTH)
				    == CAN_ID_BOTH) {
					fl_connection_flag_U8 =
					    RX_BOTH_STD_EXT_MSG;
				} else
				    if ((CAN_CH1_connect_flags &
					 CAN_29BIT_ID) == CAN_29BIT_ID) {
					fl_connection_flag_U8 = RX_ONLY_EXT_MSG;
				} else
				    if ((CAN_CH1_connect_flags &
					 CAN_29BIT_ID) == 0) {
					fl_connection_flag_U8 = RX_ONLY_STD_MSG;
				} else {
					fl_status_U8 = ERR_INVALID_FLAGS;
				}
				if (fl_status_U8 != ERR_INVALID_FLAGS) {
					fl_status_U8 =
					    CAN_Mid_Enable(CAN_CH2,
							   fl_connection_flag_U8);
					if (fl_status_U8 == STATUS_NOERROR) {
						l_connected_channels++;
						l_connected_channel_2 = 1;
					}
					DBG("HFCP DEBUG: CAN_FD CAN2 Enable Communication Cmd");
				} else {
					/* Do nothing as fl_status_U8 has been modified */
				}
			} else {
				/* Do nothing as fl_status_U8 has been modified */
			}
			if (fl_status_U8 == STATUS_NOERROR) {
				led_set(LED_LINK_LED, LED_ON);
				CAN_CH1_active = 1;
			}
#if 0
			fl_USB_tx_data_U8A[0] = proto_id;
			fl_USB_tx_data_U8A[1] = CAN_EnableComm_ACK;
			fl_USB_tx_data_U8A[3] = fl_status_U8;
			//Garuda_Tx_data_on_USB(&fl_USB_tx_data_U8A[0],4,DONT_RELEASE);
			(void)host_write((void *)fl_USB_tx_data_U8A, 4);
#endif
			((hfcpReq_t *) pbuf)->command = CAN_FD_EnableComm_ACK;
			pRsp->status = fl_status_U8;
			(void)host_write((void *)pbuf,
					   hfcp_proto_cmd_len +
					   sizeof(pRsp->status));

		} else if (proto_id == CAN_FD_PS) {
			/* CAN_FD_PS channel select */
			ps_baud_rate = hfcpreq->u.can_enable.baudrate;
			CAN_connect_flags = hfcpreq->u.can_enable.conn_flags;
			CAN_CH1_connect_flags =
			    hfcpreq->u.can_enable.conn_flags;

			((hfcpReq_t *) pbuf)->command = CAN_FD_EnableComm_ACK;
			pRsp->status = STATUS_NOERROR;	//channel is being selected in IOCTL
			led_set(LED_LINK_LED, LED_ON);
			(void)host_write((void *)pbuf,
					   hfcp_proto_cmd_len +
					   sizeof(pRsp->status));
		} else {
#if 0
			/* ERROR: It should never enter here */
			fl_USB_tx_data_U8A[0] = buffer[0];
			fl_USB_tx_data_U8A[1] = CAN_FD_EnableComm_ACK;
			fl_USB_tx_data_U8A[2] = ERR_INVALID_PROTOCOL_ID;
			(void)host_write((void *)fl_USB_tx_data_U8A, 3);
#endif
			((hfcpReq_t *) pbuf)->command = CAN_FD_EnableComm_ACK;
			pRsp->status = ERR_INVALID_PROTOCOL_ID;
			(void)host_write((void *)pbuf,
					   hfcp_proto_cmd_len +
					   sizeof(pRsp->status));
		}
		break;
	case CAN_FD_DisableComm:
		if ((proto_id == CAN_FD_PROTOCOL_ID) ||
		    (proto_id == ISO15765_FD_PROTO_ID)) {
			/* Handle CAN-CH1 */
			fl_channel_no_U8 = CAN_CH1;
			CAN_or_ISO = 0;
			CAN_FD_or_ISO15765_FD = 0;
			CAN1_Vehicle_STMinValue = 0;
			J2534_ClearAllFilter(CAN_FD_PROTOCOL_ID);
			suspend_pmsg(CAN_FD_PROTOCOL_ID);
			DBG("HFCP DEBUG : CAN Disable Communication Cmd\n");
			fl_status_U8 = CAN_Mid_Disable(fl_channel_no_U8);
			if (fl_status_U8 == STATUS_NOERROR) {
				led_set(LED_LINK_LED, LED_OFF);
				CAN_active = 0;
				if (l_connected_channels) {
					l_connected_channels--;
				}
				l_connected_channel_1 = 0;
			}
			CAN_connect_flags = 0;
#if 0
			fl_USB_tx_data_U8A[0] = proto_id;
			fl_USB_tx_data_U8A[1] = CAN_FD_DisableComm_ACK;
			fl_USB_tx_data_U8A[2] = fl_status_U8;

			(void)host_write((void *)fl_USB_tx_data_U8A, 3);
#endif
			((hfcpReq_t *) pbuf)->command = CAN_FD_DisableComm_ACK;
			pRsp->status = fl_status_U8;
			(void)host_write((void *)pbuf,
					   hfcp_proto_cmd_len +
					   sizeof(pRsp->status));

		} else if ((proto_id == CAN_FD_CH1_PROTO_ID) ||
			   (proto_id == ISO15765_FD_CH1_PROTO_ID)) {
			/* Handle CAN-CH2 */
			fl_channel_no_U8 = CAN_CH2;
			CAN1_or_ISO1 = 0;
			CANCH1_or_ISO15765CH1_J1939CH1 = 0;
			CAN2_vehicle_STMinValue = 0;
			suspend_pmsg(CAN_CH1_PROTO_ID);

			fl_status_U8 = CAN_Mid_Disable(fl_channel_no_U8);
			if (fl_status_U8 == STATUS_NOERROR) {
				CAN_CH1_active = 0;
				led_set(LED_LINK_LED, LED_OFF);
				if (l_connected_channels) {
					l_connected_channels--;
				}
				l_connected_channel_2 = 0;
			}
			CAN_CH1_connect_flags = 0;
#if 0
			fl_USB_tx_data_U8A[0] = proto_id;
			fl_USB_tx_data_U8A[1] = CAN_FD_DisableComm_ACK;
			fl_USB_tx_data_U8A[2] = fl_status_U8;
			(void)host_write((void *)fl_USB_tx_data_U8A, 3);
#endif
			((hfcpReq_t *) pbuf)->command = CAN_FD_DisableComm_ACK;
			pRsp->status = fl_status_U8;
			(void)host_write((void *)pbuf,
					   hfcp_proto_cmd_len +
					   sizeof(pRsp->status));

		} else if (proto_id == CAN_FD_PS) {
			/*handle CAN_FD_PS channel select */
			if (CAN_FD_PS_chan == CAN_CH1) {
				CAN_or_ISO = 0;
				CAN_FD_or_ISO15765_FD = 0;
				CAN1_Vehicle_STMinValue = 0;
			} else if (CAN_FD_PS_chan == CAN_CH2) {
				CAN1_or_ISO1 = 0;
				CANCH1_or_ISO15765CH1_J1939CH1 = 0;
				CAN2_vehicle_STMinValue = 0;
			}
			suspend_pmsg(CAN_FD_PS_chan);

			fl_status_U8 = CAN_Mid_Disable(CAN_FD_PS_chan);
			if (fl_status_U8 == STATUS_NOERROR) {

				if (CAN_FD_PS_chan == CAN_CH1) {
					CAN_active = 0;
					led_set(LED_LINK_LED, LED_OFF);
					CAN_connect_flags = 0;
				} else if (CAN_FD_PS_chan == CAN_CH2) {
					CAN_CH1_active = 0;
					led_set(LED_LINK_LED, LED_OFF);
					CAN_CH1_connect_flags = 0;
				}

				if (l_connected_channels) {
					l_connected_channels--;
				}
				l_connected_channel_2 = 0;
			}
		} else {
			/* ERROR: It should never enter here */
#if 0
			fl_USB_tx_data_U8A[0] = buffer[0];
			fl_USB_tx_data_U8A[1] = CAN_FD_DisableComm_ACK;
			fl_USB_tx_data_U8A[2] = ERR_INVALID_PROTOCOL_ID;
			(void)host_write((void *)fl_USB_tx_data_U8A, 3);
#endif
			((hfcpReq_t *) pbuf)->command = CAN_FD_DisableComm_ACK;
			pRsp->status = ERR_INVALID_PROTOCOL_ID;
			(void)host_write((void *)pbuf,
					   hfcp_proto_cmd_len +
					   sizeof(pRsp->status));

		}
		break;
	case CAN_FD_IOCTL_COMMAND:
		process_CAN_FD_IOCTL_cmd(buffer);
		break;
	case CAN_FD_Send_msg:
		switch (proto_id) {
		case CAN_FD_PROTOCOL_ID:
		case ISO15765_FD_PROTO_ID:
			ch_id = 1;
			break;
		case CAN_FD_CH1_PROTO_ID:
		case ISO15765_FD_CH1_PROTO_ID:
			ch_id = 2;
			break;
		case CAN_FD_PS:
			ch_id = CAN_FD_PS_chan;
			break;
		default:
			/* Error */
		}
		HFCPInitCount++;
		if (hfcpreq->u.can_sndmsg.seg_num == 0) {
			data_len = hfcpreq->u.can_sndmsg.dlen;
			memset((void *)&frame, 0, sizeof(frame));
			if ((data_len & 0xC000) == 0x0000) {
				/* Mode 0 */
				DBG("Inside mode 0 (0x000)\n");
				((ch_id == 1) ? (CAN1_Vehicle_STMinValue = 0)
				 : (CAN2_vehicle_STMinValue = 0));
				data_len &= 0x3FFF;
				DBG("data_len &= 0x3FFF=%x\n", data_len);
				/* fl_tx_flags = hfcpreq->u.can_sndmsg_m1.tx_flags; *//* need to implement for extended. */
				frame.can_id =
				    CANID_FROM_DATA(&
						    (hfcpreq->u.can_sndmsg.
						     canid));
				frame.len = data_len - 4;
				memcpy(&frame.data[0],
				       hfcpreq->u.can_sndmsg.buf, frame.len);
				if (CAN_EXT_MSG ==
				    (hfcpreq->u.can_sndmsg.
				     tx_flags & CAN_EXT_MSG)) {
					/* Extended Frame */
					frame.can_id &= CAN_EFF_MASK;
					frame.can_id |= CAN_EFF_FLAG;
				} else {
					/* Standard Frame */
					frame.can_id &= CAN_SFF_MASK;
				}
				if (ch_id == 1) {
					DBG("frame.len=%d\n", frame.len);
					fl_Q_Status =
					    CAN_FD_Mid_Transmit(CANFD_CH1,
								&frame);
				} else if (ch_id == 2) {
					fl_Q_Status =
					    CAN_FD_Mid_Transmit(CANFD_CH2,
								&frame);
				}
				/* TODO: fl_Q_Status is used unitialized */
				if (fl_Q_Status == MID_FAIL) {
					DBG("CAN_FD_Mid_Transmit - Fail\n");
					/* Message lost as the Buffer is full */
#if 0
					fl_USB_tx_data_U8A[0] = proto_id;
					fl_USB_tx_data_U8A[1] =
					    CAN_FD_Send_msg_ACK;
					fl_USB_tx_data_U8A[2] = hfcpreq->u.can_sndmsg.seg_num;	/* Segment number */
					fl_USB_tx_data_U8A[3] = ERR_BUFFER_FULL;
					//Garuda_Tx_data_on_USB(&fl_USB_tx_data_U8A[0],4,DONT_RELEASE);
					(void)host_write((void *)
							   fl_USB_tx_data_U8A,
							   4);
#endif
					((hfcpReq_t *) pbuf)->command =
					    CAN_FD_Send_msg_ACK;
					pRsp->sndmsg_rsp.seg_num = hfcpreq->u.can_sndmsg.seg_num;	/* Segment number */
					pRsp->sndmsg_rsp.status =
					    ERR_BUFFER_FULL;
					(void)host_write((void *)pbuf,
							   hfcp_proto_cmd_len
							   +
							   sizeof
							   (pRsp->sndmsg_rsp));
				} else {
					DBG("CAN_FD_Mid_Transmit - Pass\n");
					/* Clear the USB buffer as it has been processed */
#if 0
					fl_USB_tx_data_U8A[0] = proto_id;
					fl_USB_tx_data_U8A[1] =
					    CAN_FD_Send_msg_ACK;
					fl_USB_tx_data_U8A[2] =
					    hfcpreq->u.can_sndmsg.seg_num;
					fl_USB_tx_data_U8A[3] = fl_status_U8;
#endif
					((hfcpReq_t *) pbuf)->command =
					    CAN_FD_Send_msg_ACK;
					pRsp->sndmsg_rsp.seg_num = hfcpreq->u.can_sndmsg.seg_num;	/* Segment number */
					pRsp->sndmsg_rsp.status = fl_status_U8;
					pRsp->sndmsg_rsp.timestamp =
					    current_time_stamp;
					(void)host_write((void *)pbuf,
							   hfcp_proto_cmd_len
							   +
							   sizeof
							   (pRsp->sndmsg_rsp));

#if 0
					if (ch_id == 1) {
						// get_time_stamp(&current_time_stamp);
					} else {
						// get_time_stamp(&current_time_stamp);
					}
					fl_USB_tx_data_U8A[4] =
					    (uint8_t) ((current_time_stamp)
						       & 0xFF);
					fl_USB_tx_data_U8A[5] =
					    (uint8_t) ((current_time_stamp >> 8)
						       & 0xFF);
					fl_USB_tx_data_U8A[6] =
					    (uint8_t) ((current_time_stamp >>
							16) & 0xFF);
					fl_USB_tx_data_U8A[7] =
					    (uint8_t) ((current_time_stamp >>
							24) & 0xFF);

#if GARUDA_USB_DEBUG
					if (ch_id == 1) {
						Garuda_Debug.CAN_Tx_Progress =
						    getCAN1TXStatus();
						memcpy
						    (&fl_USB_tx_data_U8A
						     [20],
						     &Garuda_Debug,
						     sizeof(Garuda_Debug_Info));
					} else {
						Garuda_Debug_CAN2.
						    CAN_Tx_Progress =
						    getCAN2TXStatus();
						memcpy(&fl_USB_tx_data_U8A[20],
						       &Garuda_Debug_CAN2,
						       sizeof
						       (Garuda_Debug_Info));
					}
#endif
					(void)host_write((void *)
							   fl_USB_tx_data_U8A,
							   8);
#endif
				}
			} else if ((data_len & 0xC000) == 0x4000) {
				/* Mode 1 */
				((ch_id == 1) ? (CAN1_Vehicle_STMinValue = 0)
				 : (CAN2_vehicle_STMinValue = 0));
				stored_tx_flags_CAN =
				    hfcpreq->u.can_sndmsg_m1.tx_flags;
				stored_msg_id_CAN =
				    hfcpreq->u.can_sndmsg_m1.canid;
				stored_msg_id_CAN =
				    swap_uint32(stored_msg_id_CAN);
				if (ch_id == 1) {
					frame.can_id = stored_msg_id_CAN;
				} else {
					stored_msg_id_CAN_CH1 =
					    stored_msg_id_CAN;
					frame.can_id = stored_msg_id_CAN_CH1;
				}
				data_len &= 0x3FFF;
				frame.len = data_len - 4;
				DBG("in mode 1 (0x4000), data_len = 0x%x\n",
				    data_len);
				memcpy(&frame.data[0],
				       hfcpreq->u.can_sndmsg_m1.buf, frame.len);
				if (CAN_EXT_MSG ==
				    (stored_tx_flags_CAN & CAN_EXT_MSG)) {
					/* Extended Frame */
					frame.can_id &= CAN_EFF_MASK;
					frame.can_id |= CAN_EFF_FLAG;
				} else {
					/* Standard Frame */
					frame.can_id &= CAN_SFF_MASK;
				}
				if (ch_id == 1) {
					DBG("frame.len=%d\n", frame.len);
					fl_Q_Status =
					    CAN_FD_Mid_Transmit(CANFD_CH1,
								&frame);
				} else if (ch_id == 2) {
					fl_Q_Status =
					    CAN_FD_Mid_Transmit(CANFD_CH2,
								&frame);
				}
				if (fl_Q_Status == MID_FAIL) {
					DBG("CAN_FD_Mid_Transmit - Fail\n");
					/* Message lost as the Buffer is full */
#if 0
					fl_USB_tx_data_U8A[0] = proto_id;
					fl_USB_tx_data_U8A[1] =
					    CAN_FD_Send_msg_ACK;
					fl_USB_tx_data_U8A[2] = hfcpreq->u.can_sndmsg_m1.seg_num;	/* Segment number */
					fl_USB_tx_data_U8A[3] = ERR_BUFFER_FULL;
					(void)host_write((void *)
							   fl_USB_tx_data_U8A,
							   4);
#endif
					((hfcpReq_t *) pbuf)->command =
					    CAN_FD_Send_msg_ACK;
					pRsp->sndmsg_rsp.seg_num = hfcpreq->u.can_sndmsg_m1.seg_num;	/* Segment number */
					pRsp->sndmsg_rsp.status =
					    ERR_BUFFER_FULL;
					(void)host_write((void *)pbuf,
							   hfcp_proto_cmd_len
							   +
							   sizeof
							   (pRsp->sndmsg_rsp));
				} else {
					DBG("CAN_FD_Mid_Transmit - Pass\n");
					/* Clear the USB buffer as it has been processed */
#if 0
					fl_USB_tx_data_U8A[0] = proto_id;
					fl_USB_tx_data_U8A[1] =
					    CAN_FD_Send_msg_ACK;
					fl_USB_tx_data_U8A[2] = hfcpreq->u.can_sndmsg_m1.seg_num;	/* Segment number */
					fl_USB_tx_data_U8A[3] = fl_status_U8;
					if (ch_id == 1) {
						// get_time_stamp(&current_time_stamp);
					} else {
						// get_time_stamp(&current_time_stamp);
					}
					fl_USB_tx_data_U8A[4] =
					    (uint8_t) ((current_time_stamp)
						       & 0xFF);
					fl_USB_tx_data_U8A[5] =
					    (uint8_t) ((current_time_stamp >> 8)
						       & 0xFF);
					fl_USB_tx_data_U8A[6] =
					    (uint8_t) ((current_time_stamp >>
							16) & 0xFF);
					fl_USB_tx_data_U8A[7] =
					    (uint8_t) ((current_time_stamp >>
							24) & 0xFF);

#if GARUDA_USB_DEBUG
					if (ch_id == 1) {
						Garuda_Debug.CAN_Tx_Progress =
						    getCAN1TXStatus();
						memcpy
						    (&fl_USB_tx_data_U8A
						     [20],
						     &Garuda_Debug,
						     sizeof(Garuda_Debug_Info));
					} else {
						Garuda_Debug_CAN2.
						    CAN_Tx_Progress =
						    getCAN2TXStatus();
						memcpy(&fl_USB_tx_data_U8A[20],
						       &Garuda_Debug_CAN2,
						       sizeof
						       (Garuda_Debug_Info));
					}
#endif
					//Garuda_Tx_data_on_USB(&fl_USB_tx_data_U8A[0],8,DONT_RELEASE);
					(void)host_write((void *)
							   fl_USB_tx_data_U8A,
							   8);
#endif
					((hfcpReq_t *) pbuf)->command =
					    CAN_FD_Send_msg_ACK;
					pRsp->sndmsg_rsp.seg_num = hfcpreq->u.can_sndmsg_m1.seg_num;	/* Segment number */
					pRsp->sndmsg_rsp.status = fl_status_U8;
					pRsp->sndmsg_rsp.timestamp =
					    current_time_stamp;
					(void)host_write((void *)pbuf,
							   hfcp_proto_cmd_len
							   +
							   sizeof
							   (pRsp->sndmsg_rsp));
				}
			} else if ((data_len & 0xC000) == 0x8000) {
				/* Mode 2 */
				DBG("Inside mode 2 (0x8000)\n");
				((ch_id ==
				  1) ? (CAN1_Vehicle_STMinValue =
					hfcpreq->u.can_sndmsg_m2.STMinValue)
				 : (CAN2_vehicle_STMinValue =
				    hfcpreq->u.can_sndmsg_m2.STMinValue));

#if 0
				if (HFCPComMode == HFCP_COM_MODE_USB) {
					Ack_Required = 1;
				} else if (HFCPComMode == HFCP_COM_MODE_WIFI) {
					Ack_Required = 1;
					    // hfcpreq->u.can_sndmsg_m2.ack_req;
				}
#endif

				Ack_Required = 1;
				
				if (ch_id == 1) {
					frame.can_id = stored_msg_id_CAN;
				} else {
					frame.can_id = stored_msg_id_CAN_CH1;
				}
				data_len &= 0x3FFF;
				DBG("data_len = %d\nIf data_len <= 448 then only it will parse.\n", data_len);
				/*              data_len = buffer[4] & 0x0F;
				   data_len =(data_len << 8) | buffer[3]; //data_len &= 0x3FFF;//should work */

				if (data_len <= 448) {	      /** TODO: what is this constant, use a macro */
					if ((data_len % fd_frame_length) != 0) {
						msg_cnt =
						    (data_len /
						     fd_frame_length) + 1;
					} else {
						msg_cnt =
						    (data_len /
						     fd_frame_length);
					}
					base_indx = 0;
					DBG("No. of message is msg_cnt = %d\n",
					    msg_cnt);
					while (msg_cnt--) {
						//fl_CAN_Tx_msg_S.rtr = 0;
						/* Copy data length */
						if (data_len >= fd_frame_length) {
							data_len -=
							    fd_frame_length;
							frame.len =
							    fd_frame_length;
						} else {
							frame.len = data_len;
							data_len = 0;
						}
						/* copy all data bytes into buffer */
						memcpy(&frame.data[0],
						       &hfcpreq->u.
						       can_sndmsg_m2.
						       buf[base_indx],
						       CAN_DL_MIN_SIZE
						       (frame.len,
							sizeof(frame.data)));
						if (ch_id == 1) {
							//fl_Q_Status = CAN_FD_Mid_Transmit(CANFD_CH1, &frame);
							if (CAN1_Vehicle_STMinValue) {
									fl_Q_Status = mode2_queue_add_to_queue(CANFD_CH1, (char *)&frame, CAN1_Vehicle_STMinValue);
								} else {
									fl_Q_Status = CAN_FD_Mid_Transmit(CANFD_CH1, &frame);
								}
						} else if (ch_id == 2) {
							//fl_Q_Status = CAN_FD_Mid_Transmpit(CANFD_CH2, &frame);
							if (CAN2_vehicle_STMinValue) {
									fl_Q_Status = mode2_queue_add_to_queue(CANFD_CH2, (char *)&frame, CAN2_vehicle_STMinValue);
								} else {
									fl_Q_Status = CAN_FD_Mid_Transmit(CANFD_CH2, &frame);
								}
						}
						if (MID_FAIL != fl_Q_Status) {
							DBG("CAN_FD_Mid_Transmit - Pass\n");
							base_indx +=
							    fd_frame_length;
						} else {
							DBG("CAN_FD_Mid_Transmit - Fail\n");
							/* Do nothing */
							/* Note that the "while" in this "else if" is blocking.
							   i.e. till it writes all the frames to CAN interface it will
							   not exit. A queue strategy is to be implemented such
							   that there is always space for 7 frames.
							 */
						}
					}	/* while */
				} else {
					fl_status_U8 = ERR_FAILED;
					DBG("Mode 2 Parsing failed.\n");
				}

				/* Clear the USB buffer as it has been processed */
				if (Ack_Required == TRUE) {
#if 0
					fl_USB_tx_data_U8A[0] = proto_id;
					fl_USB_tx_data_U8A[1] =
					    CAN_FD_Send_msg_ACK;
					fl_USB_tx_data_U8A[2] = hfcpreq->u.can_sndmsg_m2.seg_num;	/* Segment number */
					fl_USB_tx_data_U8A[3] = fl_status_U8;
					if (ch_id == 1) {
						//get_time_stamp(&current_time_stamp);
					} else {
						//get_data_logging_time_stamp(&current_time_stamp);
						//get_time_stamp(&current_time_stamp);
					}
					fl_USB_tx_data_U8A[4] =
					    (uint8_t) ((current_time_stamp)
						       & 0xFF);
					fl_USB_tx_data_U8A[5] =
					    (uint8_t) ((current_time_stamp >> 8)
						       & 0xFF);
					fl_USB_tx_data_U8A[6] =
					    (uint8_t) ((current_time_stamp >>
							16) & 0xFF);
					fl_USB_tx_data_U8A[7] =
					    (uint8_t) ((current_time_stamp >>
							24) & 0xFF);
#if GARUDA_USB_DEBUG
					if (ch_id == 1) {
						Garuda_Debug.CAN_Tx_Progress =
						    getCAN1TXStatus();
						memcpy
						    (&fl_USB_tx_data_U8A
						     [20],
						     &Garuda_Debug,
						     sizeof(Garuda_Debug_Info));
					} else {
						Garuda_Debug_CAN2.
						    CAN_Tx_Progress =
						    getCAN2TXStatus();
						memcpy(&fl_USB_tx_data_U8A[20],
						       &Garuda_Debug_CAN2,
						       sizeof
						       (Garuda_Debug_Info));
					}
#endif
					//Garuda_Tx_data_on_USB(&fl_USB_tx_data_U8A[0],8,DONT_RELEASE);
					(void)host_write((void *)
							   fl_USB_tx_data_U8A,
							   8);
#endif
					((hfcpReq_t *) pbuf)->command =
					    CAN_FD_Send_msg_ACK;
					pRsp->sndmsg_rsp.seg_num = hfcpreq->u.can_sndmsg_m2.seg_num;	/* Segment number */
					pRsp->sndmsg_rsp.status = fl_status_U8;
					pRsp->sndmsg_rsp.timestamp =
					    current_time_stamp;
					(void)host_write((void *)pbuf,
							   hfcp_proto_cmd_len
							   +
							   sizeof
							   (pRsp->sndmsg_rsp));

				}
			} else if ((data_len & 0xC000) == 0xC000) {
				/* Mode 3 */
				struct mode_data *p;

				DBG("Inside mode 3 (0xC000)\n");

				p = (struct mode_data *)hfcpreq->u.
				    can_sndmsg_m3.data;

				base_indx = 5;
				msg_cnt = (uint8_t) (data_len & 0x3FFF);
				DBG("In Mode 3: msg_cnt=%x %d\n",
				    msg_cnt, msg_cnt);
				while (msg_cnt--) {
					fl_msg_id = CANID_FROM_DATA(&p->canid);
					//fl_tx_flags = hfcpreq->u.can_sndmsg_m1.tx_flags;
					data_len = p->dlen;
					memset((void *)&frame, 0,
					       sizeof(frame));
					frame.can_id = fl_msg_id;
					frame.len = data_len;
					memcpy(&frame.data[0], p->buf,
					       frame.len);
					if (CAN_EXT_MSG ==
					    (hfcpreq->u.can_sndmsg.
					     tx_flags & CAN_EXT_MSG)) {
						/* Extended Frame */
						frame.can_id &= CAN_EFF_MASK;
						frame.can_id |= CAN_EFF_FLAG;
					} else {
						/* Standard Frame */
						frame.can_id &= CAN_SFF_MASK;
					}
					do {
						if (ch_id == 1) {
							fl_Q_Status =
							    CAN_FD_Mid_Transmit
							    (CANFD_CH1, &frame);
						} else if (ch_id == 2) {
							fl_Q_Status =
							    CAN_FD_Mid_Transmit
							    (CANFD_CH2, &frame);
						}
					} while (fl_Q_Status ==
						 ERR_BUFFER_FULL);

					base_indx += data_len;
					p = (struct mode_data
					     *)((uint8_t *) p + data_len);
				}	// while()
				/* Clear the USB buffer as it has been processed */
#if 0
				fl_USB_tx_data_U8A[0] = proto_id;
				fl_USB_tx_data_U8A[1] = CAN_FD_Send_msg_ACK;
				fl_USB_tx_data_U8A[2] = hfcpreq->u.can_sndmsg_m3.seg_num;	/* Segment number */
				fl_USB_tx_data_U8A[3] = fl_status_U8;
				if (ch_id == 1) {
					//get_time_stamp(&current_time_stamp);
				} else {
					// get_time_stamp(&current_time_stamp);
				}
				fl_USB_tx_data_U8A[4] =
				    (uint8_t) ((current_time_stamp) & 0xFF);
				fl_USB_tx_data_U8A[5] =
				    (uint8_t) ((current_time_stamp >> 8)
					       & 0xFF);
				fl_USB_tx_data_U8A[6] =
				    (uint8_t) ((current_time_stamp >>
						16) & 0xFF);
				fl_USB_tx_data_U8A[7] =
				    (uint8_t) ((current_time_stamp >>
						24) & 0xFF);

#if GARUDA_USB_DEBUG
				if (ch_id == 1) {
					Garuda_Debug.CAN_Tx_Progress =
					    getCAN1TXStatus();
					memcpy(&fl_USB_tx_data_U8A[20],
					       &Garuda_Debug,
					       sizeof(Garuda_Debug_Info));
				} else {
					Garuda_Debug_CAN2.CAN_Tx_Progress =
					    getCAN2TXStatus();
					memcpy(&fl_USB_tx_data_U8A[20],
					       &Garuda_Debug_CAN2,
					       sizeof(Garuda_Debug_Info));
				}
#endif
				//Garuda_Tx_data_on_USB(&fl_USB_tx_data_U8A[0],8,DONT_RELEASE);
				(void)host_write((void *)
						   fl_USB_tx_data_U8A, 8);
#endif
				((hfcpReq_t *) pbuf)->command =
				    CAN_FD_Send_msg_ACK;
				pRsp->sndmsg_rsp.seg_num =
				    hfcpreq->u.can_sndmsg_m3.seg_num;
				pRsp->sndmsg_rsp.status = fl_status_U8;
				pRsp->sndmsg_rsp.timestamp = current_time_stamp;
				(void)host_write((void *)pbuf,
						   hfcp_proto_cmd_len +
						   sizeof(pRsp->sndmsg_rsp));
			}
			/* ERROR : Illegal Mode */
			else {
				/* Clear the USB buffer as it has been processed */
#if 0
				fl_USB_tx_data_U8A[0] = proto_id;
				fl_USB_tx_data_U8A[1] = CAN_FD_Send_msg_ACK;
				fl_USB_tx_data_U8A[2] = hfcpreq->u.can_sndmsg_m3.seg_num;	/* Segment number */
				fl_USB_tx_data_U8A[3] = ERR_FAILED;
				//Garuda_Tx_data_on_USB(&fl_USB_tx_data_U8A[0],3,DONT_RELEASE);
				(void)host_write((void *)fl_USB_tx_data_U8A, 3);	/* TODO: BUG write 4 bytes */
#endif
				((hfcpReq_t *) pbuf)->command =
				    CAN_FD_Send_msg_ACK;
				pRsp->sndmsg_rsp.seg_num = hfcpreq->u.can_sndmsg_m3.seg_num;	/* Segment number */
				pRsp->sndmsg_rsp.status = ERR_FAILED;
				(void)host_write((void *)pbuf,
						   hfcp_proto_cmd_len +
						   sizeof(pRsp->sndmsg_rsp));
			}
		} else {
			/*
			   fl_USB_tx_data_U8A[0] = proto_id;
			   fl_USB_tx_data_U8A[1] = CAN_FD_Send_msg_ACK;
			   fl_USB_tx_data_U8A[2] = hfcpreq->u.can_sndmsg_m3.seg_num; // Segment number 
			   fl_USB_tx_data_U8A[3] = ERR_FAILED;
			   (void) host_write((void *)fl_USB_tx_data_U8A, 4); */
			((hfcpReq_t *) pbuf)->command = CAN_FD_Send_msg_ACK;
			pRsp->sndmsg_rsp.seg_num = hfcpreq->u.can_sndmsg_m3.seg_num;	/* Segment number */
			pRsp->sndmsg_rsp.status = ERR_FAILED;
			(void)host_write((void *)pbuf,
					   hfcp_proto_cmd_len +
					   sizeof(pRsp->sndmsg_rsp));
		}
		break;
	case Start_FD_msg_filter:
		if ((proto_id == ISO15765_FD_PROTO_ID) ||
		    (proto_id == CAN_FD_PROTOCOL_ID)) {
			fl_Protocol_ID_U8 = CAN_FD_PROTOCOL_ID;

		} else if ((proto_id == ISO15765_FD_CH1_PROTO_ID) ||
			   (proto_id == CAN_FD_CH1_PROTO_ID)) {
			fl_Protocol_ID_U8 = CAN_FD_CH1_PROTO_ID;
		}
		/* TODO: fl_Protocol_ID_U8 is used uninitilized */
		DBG("Passing to ConfigFilter:Protocol_ID=0x%x\tfl_Protocol_ID=0x%x\n", fl_filter_config.Protocol_ID, fl_Protocol_ID_U8);
		fl_filter_config.Protocol_ID = fl_Protocol_ID_U8;
		fl_filter_config.filterType =
		    (J2534_filterType_t) (hfcpreq->u.can_msg_filter.filtertype);
		plv = (lv_t *) (hfcpreq->u.can_msg_filter.buf);
		fl_filter_config.MaskLen = plv->len;
		for (i = 0; i < plv->len; i++) {
			fl_filter_config.maskMsg[i] = plv->buf[i];
		}
		plv =
		    (lv_t *) ((hfcpreq->u.can_msg_filter.buf) +
			      sizeof(plv->len) + plv->len);
		fl_filter_config.PatternLen = plv->len;
		for (i = 0; i < plv->len; i++) {
			fl_filter_config.patternMsg[i] = plv->buf[i];
		}
		fl_filt_config_status =
		    J2534_ConfigFilter(&fl_filter_config, &fl_FilterID);
		((hfcpReq_t *) pbuf)->command = Start_FD_msg_filter_ACK;
		pRsp->msg_fil.filterid = fl_FilterID;

		if (fl_filt_config_status != J2534_NO_ERROR) {
			DBG("FD_msg_filter Error.\n");
			if (fl_filt_config_status == J2534_FLT_NOT_FREE) {
				fl_status_U8 = ERR_EXCEEDED_LIMIT;
			} else {
				fl_status_U8 = ERR_FAILED;
			}
#if 0
			fl_USB_tx_data_U8A[0] = proto_id;
			fl_USB_tx_data_U8A[1] = Start_msg_filter_ACK;
			fl_USB_tx_data_U8A[2] = fl_status_U8;
			fl_USB_tx_data_U8A[3] = fl_FilterID;
			//Garuda_Tx_data_on_USB(&fl_USB_tx_data_U8A[0],5,DONT_RELEASE);
			(void)host_write((void *)fl_USB_tx_data_U8A, 5);	/* TODO: write 4 bytes */
#endif
			pRsp->msg_fil.status = fl_status_U8;
		} else {
			DBG("FD_msg_filter NO Error.\n");
#if 0
			fl_USB_tx_data_U8A[0] = proto_id;
			fl_USB_tx_data_U8A[1] = Start_msg_filter_ACK;
			fl_USB_tx_data_U8A[2] = STATUS_NOERROR;
			fl_USB_tx_data_U8A[3] = fl_FilterID;

			//Garuda_Tx_data_on_USB(&fl_USB_tx_data_U8A[0],5,DONT_RELEASE);
			(void)host_write((void *)fl_USB_tx_data_U8A, 5);	/* TODO: Write 4 bytes only */
#endif
			pRsp->msg_fil.status = STATUS_NOERROR;
		}
		(void)host_write((void *)pbuf,
				   hfcp_proto_cmd_len + sizeof(pRsp->msg_fil));
		break;
	case Stop_FD_msg_filter:
		switch (proto_id) {
		case ISO15765_FD_PROTO_ID:
		case CAN_FD_PROTOCOL_ID:
			fl_Protocol_ID_U8 = CAN_FD_PROTOCOL_ID;
			break;
		case ISO15765_FD_CH1_PROTO_ID:
		case CAN_FD_CH1_PROTO_ID:
			fl_Protocol_ID_U8 = CAN_FD_CH1_PROTO_ID;
			break;
		default:
			DBG("Invalid proto_id");
			/* error */
		}
		fl_FilterID = hfcpreq->u.can_stpmsg_filter.filterid;
		DBG("Filter_ID=0x%x\tProtocol_ID=0x%x\n", fl_FilterID,
		    fl_Protocol_ID_U8);
		fl_filt_stop_status =
		    J2534_ClearFilter(fl_FilterID, fl_Protocol_ID_U8);

		((hfcpReq_t *) pbuf)->command = Stop_FD_msg_filter_ACK;
		pRsp->stpmsg_fil.filterid = fl_FilterID;
		if (fl_filt_stop_status != J2534_NO_ERROR) {
			if (fl_filt_stop_status == J2534_INVLD_FLTID) {
				fl_status_U8 = ERR_INVALID_FILTER_ID;
			} else {
				fl_status_U8 = ERR_FAILED;
			}
#if 0
			fl_USB_tx_data_U8A[0] = fl_Protocol_ID_U8;
			fl_USB_tx_data_U8A[1] = Stop_msg_filter_ACK;
			fl_USB_tx_data_U8A[2] = fl_status_U8;
			fl_USB_tx_data_U8A[3] = fl_FilterID;
			(void)host_write((void *)fl_USB_tx_data_U8A, 4);
#endif
			((hfcpReq_t *) pbuf)->proto_id = fl_Protocol_ID_U8;
			pRsp->stpmsg_fil.status = fl_status_U8;

		} else {
#if 0
			fl_USB_tx_data_U8A[0] = proto_id;
			fl_USB_tx_data_U8A[1] = Stop_msg_filter_ACK;
			fl_USB_tx_data_U8A[2] = STATUS_NOERROR;
			fl_USB_tx_data_U8A[3] = fl_FilterID;
			(void)host_write((void *)fl_USB_tx_data_U8A, 4);
#endif
			pRsp->msg_fil.status = STATUS_NOERROR;
		}
		(void)host_write((void *)pbuf,
				   hfcp_proto_cmd_len +
				   sizeof(pRsp->stpmsg_fil));
		DBG("HFCP DEBUG : FD Stop Mesage Filter = %d \r\n",
		    fl_status_U8);
		break;
	case handle_FD_periodic_msg:
		switch (proto_id) {
		case ISO15765_FD_PROTO_ID:
		case CAN_FD_PROTOCOL_ID:
			fl_Protocol_ID_U8 = CAN_FD_PROTOCOL_ID;
			break;
		case ISO15765_FD_CH1_PROTO_ID:
		case CAN_FD_CH1_PROTO_ID:
			fl_Protocol_ID_U8 = CAN_FD_CH1_PROTO_ID;
			break;
		default:
			DBG("Invalid proto_id");
			/* error */
		}
		periodic_msg_cmd = hfcpreq->u.can_periodic_msg.periodic_command;
		//(void) host_write((void *)pReq, 9);
		j2534_periodic_msg.protocol_id = fl_Protocol_ID_U8;
		j2534_periodic_msg.periodicity =
		    hfcpreq->u.can_periodic_msg.periodicity;
		j2534_periodic_msg.prmsg_id = hfcpreq->u.can_periodic_msg.msgid;
		j2534_periodic_msg.proto_msg.tx_flags =
		    hfcpreq->u.can_periodic_msg.tx_flags;
		if ((periodic_msg_cmd == START_NEW_PERIODIC_MSG_TXN)
		    || (periodic_msg_cmd == UPDATE_DATA_TO_MSG_ID)) {
			j2534_periodic_msg.proto_msg.length =
			    hfcpreq->u.can_periodic_msg.msglen;
		} else {
			j2534_periodic_msg.proto_msg.length = 0;
		}
		for (loop_count = 0;
		     loop_count < j2534_periodic_msg.proto_msg.length;
		     loop_count++) {
			j2534_periodic_msg.proto_msg.data[loop_count] =
			    hfcpreq->u.can_periodic_msg.buf[loop_count];
		}
#if 1
		//if(PERIODIC_SUCCESS==0)
		if (PERIODIC_SUCCESS ==
		    PERIODIC_msg_cmd(&j2534_periodic_msg, periodic_msg_cmd)) {
			fl_status_U8 = STATUS_NOERROR;
		} else {
			fl_status_U8 = ERR_FAILED;
		}
#endif
		/* Send Acknowledgment to J2534 DLL */
#if 0
		fl_USB_tx_data_U8A[0] = proto_id;
		fl_USB_tx_data_U8A[1] = handle_periodic_msg_ACK;
		fl_USB_tx_data_U8A[2] = fl_status_U8;
		fl_USB_tx_data_U8A[3] = j2534_periodic_msg.prmsg_id;
		//Garuda_Tx_data_on_USB(&fl_USB_tx_data_U8A[0],4,DONT_RELEASE);
		(void)host_write((void *)fl_USB_tx_data_U8A, 4);
#endif
		((hfcpReq_t *) pbuf)->command = handle_FD_periodic_msg_ACK;
		pRsp->pmsg.status = fl_status_U8;
		pRsp->pmsg.per_msg_id = j2534_periodic_msg.prmsg_id;
		(void)host_write((void *)pbuf,
				   hfcp_proto_cmd_len + sizeof(pRsp->pmsg));
		break;
	default:
		break;
	}
	free(pbuf);
}

void process_CAN_IOCTL_cmd(uint8_t * buff)
{
	hfcpReq_t *hfcpreq;
	hfcpResp_t *pRsp;
	uint32_t proto_id;
	void *pbuf;
	int hfcp_proto_cmd_len;

	uint8_t ch_id = 0;
	uint32_t fl_current_baudrate_U32 = 0, fl_IOCTL_CMD_U8 = 0;
	uint32_t fl_Protocol_ID_U8 = 0;
	uint8_t fl_loopback_status_U8, fl_SAM_status_U8, fl_SJW_status_U8;
	uint8_t fl_set_BR_ret_status_U8;
	uint8_t fl_filt_stopAll_status, fl_Q_clear_status;

#ifdef DEBUG_MODE
	DBG("***Inside The process_CAN_IOCTL_cmd***");
#endif

	hfcpreq = (hfcpReq_t *) buff;
	pbuf = malloc(USB_PKT_SIZE);
	if (pbuf == NULL) {
		ERR("malloc failed!");
		return;
	}
	memset(pbuf, 0, sizeof(USB_PKT_SIZE));
	proto_id = hfcpreq->proto_id;
	hfcp_proto_cmd_len = sizeof(hfcpReq_t) - sizeof(hfcpreq->u);
	((hfcpReq_t *) pbuf)->proto_id = proto_id;
	pRsp = (hfcpResp_t *) ((uint8_t *) pbuf + hfcp_proto_cmd_len);
	fl_IOCTL_CMD_U8 = hfcpreq->u.ioctl_req.command_ID;	//IOCTL Command

	DBG("***Print protocol ID fl_IOCTL_CMD_U8=0x%x***", fl_IOCTL_CMD_U8);

	if ((proto_id == CAN_PROTOCOL_ID)
	    || (proto_id == ISO15765_PROTO_ID)) {
		ch_id = 1;
	} else if ((proto_id == CAN_CH1_PROTO_ID) ||
		   (buff[0] == ISO15765_CH1_PROTO_ID)) {
		ch_id = 2;
	} else {
		//Error it will never reach here.
	}

	switch (fl_IOCTL_CMD_U8) {
#ifdef DEBUG_MODE
		DBG("***Inside IOCTL switch case***");
#endif
	case Get_config:
		DBG("***Inside CAN_IOCTL Get_config***");
#if 0
		fl_USB_tx_data_U8A[0] = buff[0];
		fl_USB_tx_data_U8A[1] = IOCTL_RESPONSE;	//IOCTL CMD ID
		fl_USB_tx_data_U8A[2] = Get_config_ACK;	//ACK for Get_config CMD
#endif
		((hfcpReq_t *) pbuf)->command = IOCTL_RESPONSE;
		pRsp->ioctl_resp.com_id = Get_config_ACK;
		if (Get_data_rate == hfcpreq->u.ioctl_req.data) {
			/*Get config parameter ID == Data rate */
			DBG("***Inside CAN_IOCTL Get_config Get_data_rate Get_data_rate=%d hfcpreq->u.ioctl_req.data=%d hfcpreq->u.ioctl_req.data**", Get_data_rate, hfcpreq->u.ioctl_req.data);
			fl_current_baudrate_U32 =
			    CAN_Mid_get_current_baudrate(ch_id);
			if (0xFFFFFFFF == fl_current_baudrate_U32) {
				//fl_USB_tx_data_U8A[3] = ERR_FAILED;
				pRsp->ioctl_resp.status = ERR_FAILED;
			} else {
				//fl_USB_tx_data_U8A[3] = STATUS_NOERROR;
				pRsp->ioctl_resp.status = STATUS_NOERROR;
			}
#if 0
			fl_USB_tx_data_U8A[4] = buff[3];	/* ACK for data rate parameter */
			fl_USB_tx_data_U8A[5] = Get_data_rate_ACK;	/* ACK for data rate parameter */
			fl_USB_tx_data_U8A[6] =
			    (uint8_t) ((fl_current_baudrate_U32) & 0xFF);
			fl_USB_tx_data_U8A[7] =
			    (uint8_t) ((fl_current_baudrate_U32 >> 8) & 0xFF);
			fl_USB_tx_data_U8A[8] =
			    (uint8_t) ((fl_current_baudrate_U32 >> 16) & 0xFF);
			fl_USB_tx_data_U8A[9] =
			    (uint8_t) ((fl_current_baudrate_U32 >> 24) & 0xFF);
#endif
			pRsp->ioctl_resp.length = hfcpreq->u.ioctl_req.length;
			pRsp->ioctl_resp.data_rate = Get_data_rate_ACK;
			pRsp->ioctl_resp.u.baudrate = fl_current_baudrate_U32;
		}

		else if (Get_loopback == hfcpreq->u.ioctl_req.data) {
			DBG("***Inside CAN_IOCTL Get_config Get_loopback***");
			DBG("Get_loopback=%d, hfcpreq->u.ioctl_req.data=%d\n",
			    Get_loopback, hfcpreq->u.ioctl_req.data);
			fl_loopback_status_U8 =
			    LowLevel_CAN_get_loopback_status(ch_id);
			/* Legacy code */
			if (ERR_INVALID_CHANNEL_ID == fl_loopback_status_U8) {
				//fl_USB_tx_data_U8A[3] = ERR_INVALID_CHANNEL_ID;
				pRsp->ioctl_resp.status =
				    ERR_INVALID_CHANNEL_ID;

			} else {
				//fl_USB_tx_data_U8A[3] = fl_loopback_status_U8;
				pRsp->ioctl_resp.status = fl_loopback_status_U8;
			}
#if 0
			fl_USB_tx_data_U8A[3] = STATUS_NOERROR;
			fl_USB_tx_data_U8A[4] = 1;	/* number of bytes */
			fl_USB_tx_data_U8A[5] = Get_loopback_ACK;	/* ACK for loopback status */
			fl_USB_tx_data_U8A[6] = fl_loopback_status_U8;	/* 0 - OFF, 1 - ON */
#endif
			pRsp->ioctl_resp.status = STATUS_NOERROR;
			pRsp->ioctl_resp.length = 1;
			pRsp->ioctl_resp.data_rate = Get_loopback_ACK;

			pRsp->ioctl_resp.u.loopback_status =
			    fl_loopback_status_U8;

		} else if (Get_sample_point == hfcpreq->u.ioctl_req.data) {
			DBG("***Inside CAN_IOCTL Get_configi Get_Sample_point***");
			DBG("Get_sample_point=%d, hfcpreq->u.ioctl_req.data=%d\n", Get_sample_point, hfcpreq->u.ioctl_req.data);
			fl_SAM_status_U8 =
			    LowLevel_CAN_get_current_sampling_mode(ch_id);

			/* Legacy code */
			if (ERR_INVALID_CHANNEL_ID == fl_SAM_status_U8) {
				//fl_USB_tx_data_U8A[3] = ERR_INVALID_CHANNEL_ID;
				pRsp->ioctl_resp.status =
				    ERR_INVALID_CHANNEL_ID;

			} else {
				//fl_USB_tx_data_U8A[3] = fl_SAM_status_U8;
				pRsp->ioctl_resp.status = fl_SAM_status_U8;
			}

			//fl_USB_tx_data_U8A[4] = buff[3];
			//fl_USB_tx_data_U8A[5] = Get_sample_point_ACK;
			pRsp->ioctl_resp.length = hfcpreq->u.ioctl_req.length;
			pRsp->ioctl_resp.data_rate = Get_sample_point_ACK;

		} else if (Get_SJW == hfcpreq->u.ioctl_req.data) {
			DBG("***Inside CAN_IOCTL Get_config Get_SJW***");
			DBG("Get_SJW=%d, hfcpreq->u.ioctl_req.data=%d",
			    Get_SJW, hfcpreq->u.ioctl_req.data);

			fl_SJW_status_U8 = LowLevel_CAN_get_current_SJW(ch_id);

			/* Legacy code */
			if (ERR_INVALID_CHANNEL_ID == fl_SJW_status_U8) {
				//fl_USB_tx_data_U8A[3] = ERR_INVALID_CHANNEL_ID;
				pRsp->ioctl_resp.status =
				    ERR_INVALID_CHANNEL_ID;
			} else {
				//fl_USB_tx_data_U8A[3] = fl_SJW_status_U8;
				pRsp->ioctl_resp.status = fl_SJW_status_U8;
			}

			//fl_USB_tx_data_U8A[4] = buff[3];
			/* fl_USB_tx_data_U8A[5] = Get_SJW_ACK; *//* ACK for data rate parameter */
			pRsp->ioctl_resp.length = hfcpreq->u.ioctl_req.length;
			pRsp->ioctl_resp.data_rate = Get_SJW_ACK;

		}

		else {
			DBG("***Inside CAN_IOCTL Get_config INVALID_IOCTL_ID***");
#if 0
			fl_USB_tx_data_U8A[3] = ERR_INVALID_IOCTL_ID;
			fl_USB_tx_data_U8A[4] = buff[3];	/* ACK for data rate parameter */
			fl_USB_tx_data_U8A[5] = fl_IOCTL_CMD_U8;	/* ACK for data rate parameter */
#endif
			pRsp->ioctl_resp.status = ERR_INVALID_IOCTL_ID;
			pRsp->ioctl_resp.length = hfcpreq->u.ioctl_req.length;
			pRsp->ioctl_resp.data_rate = fl_IOCTL_CMD_U8;
		}

		//(void) host_write((void *)fl_USB_tx_data_U8A,5);
		(void)host_write((void *)pbuf,
				   hfcp_proto_cmd_len +
				   sizeof(pRsp->ioctl_resp));
		break;
	case Set_config:
		DBG("Inside Set_config");
#if 0
		fl_USB_tx_data_U8A[0] = buff[0];
		fl_USB_tx_data_U8A[1] = IOCTL_RESPONSE;	/* IOCTL CMD ID */
		fl_USB_tx_data_U8A[2] = Set_config_ACK;	/* ACK for Get config CMD */
#endif
		((hfcpReq_t *) pbuf)->command = IOCTL_RESPONSE;
		pRsp->ioctl_resp.com_id = Set_config_ACK;
		if (Set_data_rate == hfcpreq->u.ioctl_req.data) {	/*Set config parameter ID == Data rate */
			DBG("Inside CAN_IOCTL Set_config Set_data_rate\n");
#if 0
			fl_current_baudrate_U32 =
			    fl_current_baudrate_U32 | (uint32_t)
			    buff[5];
			fl_current_baudrate_U32 =
			    fl_current_baudrate_U32 | (uint32_t)
			    buff[6] << 8;
			fl_current_baudrate_U32 =
			    fl_current_baudrate_U32 | (uint32_t)
			    buff[7] << 16;
			fl_current_baudrate_U32 =
			    fl_current_baudrate_U32 | (uint32_t)
			    buff[8] << 24;
#endif
			fl_current_baudrate_U32 =
			    hfcpreq->u.ioctl_req.un.baudrate;
			/* Added Code for Deinitilization CAN Interface.        */
			/* Initialization of CAN with set config baudrate       */
			/* Update the Baud Rate to Global Structure             */
			CAN_Mid_DeInit((ch_id == 1) ? "can0" : "can1");
			CAN_Mid_Init(((ch_id ==
				       1) ? "can0" : "can1"),
				     fl_current_baudrate_U32);
			fl_set_BR_ret_status_U8 =
			    CAN_Mid_set_baudrate
			    (fl_current_baudrate_U32, ch_id);
#if 0
			fl_USB_tx_data_U8A[3] = fl_set_BR_ret_status_U8;	/* ACK for data rate parameter */
			fl_USB_tx_data_U8A[4] = buff[3];
			fl_USB_tx_data_U8A[5] = Set_data_rate_ACK;	/* ACK for data rate parameter */
#endif
			pRsp->ioctl_resp.status = fl_set_BR_ret_status_U8;
			pRsp->ioctl_resp.length = hfcpreq->u.ioctl_req.length;
			pRsp->ioctl_resp.data_rate = Set_data_rate_ACK;
			pRsp->ioctl_resp.u.baudrate = fl_current_baudrate_U32;
		} else if (Set_loopback == hfcpreq->u.ioctl_req.data) {
			/* Set config parameter ID == loopback */
			DBG("Inside CAN_IOCTL Set_config Set_loopback");
			if (hfcpreq->u.ioctl_req.un.loopback_status == 1) {
				fl_loopback_status_U8 =
				    LowLevel_CAN_enable_loopback(ch_id);

			} else if (hfcpreq->u.ioctl_req.un.loopback_status == 0) {
				fl_loopback_status_U8 =
				    LowLevel_CAN_disable_loopback(ch_id);
			} else {
				fl_loopback_status_U8 = ERR_INVALID_IOCTL_ID;
			}
#if 0
			fl_USB_tx_data_U8A[3] = fl_loopback_status_U8;	/* ACK for data rate parameter */
			fl_USB_tx_data_U8A[4] = buff[3];
			fl_USB_tx_data_U8A[5] = Set_loopback_ACK;
#endif
			pRsp->ioctl_resp.status = fl_loopback_status_U8;
			pRsp->ioctl_resp.length = hfcpreq->u.ioctl_req.length;
			pRsp->ioctl_resp.data_rate = Set_loopback_ACK;
		} else if ((Set_sample_point == hfcpreq->u.ioctl_req.data)
			   || (Set_SJW == hfcpreq->u.ioctl_req.data)) {
			DBG("Inside CAN_IOCTL Set_config Set_sample_point");
#if 0
			fl_USB_tx_data_U8A[3] = ERR_NOT_SUPPORTED;
			fl_USB_tx_data_U8A[4] = buff[3];
			fl_USB_tx_data_U8A[5] = fl_IOCTL_CMD_U8;
#endif
			pRsp->ioctl_resp.status = ERR_NOT_SUPPORTED;
			pRsp->ioctl_resp.length = hfcpreq->u.ioctl_req.length;
			pRsp->ioctl_resp.data_rate = fl_IOCTL_CMD_U8;
		} else {
			DBG("Inside CAN_IOCTL Set_config INVALID_IOCTL_ID");

#if 0
			fl_USB_tx_data_U8A[3] = ERR_INVALID_IOCTL_ID;
			fl_USB_tx_data_U8A[4] = buff[3];	/* ACK for data rate parameter */
			fl_USB_tx_data_U8A[5] = fl_IOCTL_CMD_U8;	/* ACK for data rate parameter */
#endif
			pRsp->ioctl_resp.status = ERR_INVALID_IOCTL_ID;
			pRsp->ioctl_resp.length = hfcpreq->u.ioctl_req.length;
			pRsp->ioctl_resp.data_rate = fl_IOCTL_CMD_U8;
		}

		(void)host_write((void *)pbuf,
				   hfcp_proto_cmd_len +
				   sizeof(pRsp->ioctl_resp));
		break;
	case clear_all_msg_filters:
		DBG("Inside clear_all_msg_filters");
		if ((proto_id == ISO15765_PROTO_ID)
		    || (proto_id == CAN_PROTOCOL_ID)) {
			fl_Protocol_ID_U8 = CAN_PROTOCOL_ID;
		} else if ((proto_id == ISO15765_CH1_PROTO_ID)
			   || (proto_id == CAN_CH1_PROTO_ID)) {
			fl_Protocol_ID_U8 = CAN_CH1_PROTO_ID;
		}
		fl_filt_stopAll_status = J2534_ClearAllFilter(fl_Protocol_ID_U8);	// declare and define the function before use
		if (fl_filt_stopAll_status == J2534_NO_ERROR) {
			/* fl_USB_tx_data_U8A[3] = STATUS_NOERROR; *//* Staus No_ERROR */
			pRsp->ioctl_resp.status = STATUS_NOERROR;
		} else {
			/* fl_USB_tx_data_U8A[3] = ERR_FAILED; *//* Failed due to Wrong Protocol ID */
			pRsp->ioctl_resp.status = ERR_FAILED;
		}
#if 0
		fl_USB_tx_data_U8A[0] = buff[0];
		fl_USB_tx_data_U8A[1] = IOCTL_COMMAND;	/* IOCTL CMD ID */
		fl_USB_tx_data_U8A[2] = clear_all_msg_filters_ACK;	/* Command ID */
#endif
		((hfcpReq_t *) pbuf)->command = IOCTL_COMMAND;
		pRsp->ioctl_resp.com_id = clear_all_msg_filters_ACK;

		//(void) host_write((void *)fl_USB_tx_data_U8A,5);
		(void)host_write((void *)pbuf,
				   hfcp_proto_cmd_len +
				   sizeof(pRsp->ioctl_resp));
		break;
	case clear_all_periodic_msgs:
		DBG("Inside clear_all_periodic_msgs");
#if 0
		fl_USB_tx_data_U8A[0] = buff[0];
		fl_USB_tx_data_U8A[1] = IOCTL_COMMAND;	/* IOCTL CMD ID */
		fl_USB_tx_data_U8A[2] = clear_all_periodic_msgs_ACK;
#endif
		((hfcpReq_t *) pbuf)->command = IOCTL_COMMAND;
		pRsp->ioctl_resp.com_id = clear_all_periodic_msgs_ACK;
		if ((proto_id == ISO15765_PROTO_ID)
		    || (proto_id == CAN_PROTOCOL_ID)) {
			fl_Protocol_ID_U8 = CAN_PROTOCOL_ID;
		} else if ((proto_id == ISO15765_CH1_PROTO_ID)
			   || (proto_id == CAN_CH1_PROTO_ID)) {
			fl_Protocol_ID_U8 = CAN_CH1_PROTO_ID;
		}
		// uint32_t fl_Protocol_ID_U8 = 0;
		/* @TODO: What is this doing here? */
		suspend_pmsg(fl_Protocol_ID_U8);	// Suspend all periodic messages
		/* declare and define the function before use */
		/* fl_USB_tx_data_U8A[3] = STATUS_NOERROR; *//* Error status */
		pRsp->ioctl_resp.status = STATUS_NOERROR;
		/*((hfcpReq_t *)pbuf)->command = IOCTL_COMMAND;
		   pRsp->ioctl_resp.com_id = clear_all_periodic_msgs_ACK; */

		//(void) host_write((void *)fl_USB_tx_data_U8A,5);
		(void)host_write((void *)pbuf,
				   hfcp_proto_cmd_len +
				   sizeof(pRsp->ioctl_resp));

		break;
	case clear_tx_buffer:
		DBG("Inside clear_tx_buffer\n");

		DBG("Inside clear_tx_buffer\n");
		if ((proto_id == ISO15765_PROTO_ID)
		    || (proto_id == CAN_PROTOCOL_ID)) {
			fl_Q_clear_status = MID_PASS;
		} else if ((proto_id == ISO15765_CH1_PROTO_ID)
			   || (proto_id == CAN_CH1_PROTO_ID)) {
			fl_Q_clear_status = MID_PASS;
		}
		if (fl_Q_clear_status == MID_PASS) {
			pRsp->ioctl_resp.status = STATUS_NOERROR;	/* Staus No_ERROR */
		} else {
			pRsp->ioctl_resp.status = ERR_FAILED;	/* Failed due to Wrong Protocol ID */
		}
#if 0
		fl_USB_tx_data_U8A[0] = buff[0];
		fl_USB_tx_data_U8A[1] = IOCTL_COMMAND;	/* IOCTL CMD ID */
		fl_USB_tx_data_U8A[2] = clear_tx_buffer_ACK;	/* Command ID */
#endif
		((hfcpReq_t *) pbuf)->command = IOCTL_COMMAND;
		pRsp->ioctl_resp.com_id = clear_tx_buffer_ACK;
		(void)host_write((void *)pbuf,
				   hfcp_proto_cmd_len +
				   sizeof(pRsp->ioctl_resp));
		break;
	case clear_rx_buffer:
		DBG("Inside clear_rx_buffer\n");
		if ((proto_id == ISO15765_PROTO_ID)
		    || (proto_id == CAN_PROTOCOL_ID)) {
			fl_Q_clear_status = MID_PASS;
		} else if ((proto_id == ISO15765_CH1_PROTO_ID)
			   || (proto_id == CAN_CH1_PROTO_ID)) {
			fl_Q_clear_status = MID_PASS;
		}
		if (fl_Q_clear_status == MID_PASS) {
			pRsp->ioctl_resp.status = STATUS_NOERROR;	/* Staus No_ERROR */
		} else {
			pRsp->ioctl_resp.status = ERR_FAILED;	/* Failed due to Wrong Protocol ID */
		}
#if 0
		fl_USB_tx_data_U8A[0] = buff[0];
		fl_USB_tx_data_U8A[1] = IOCTL_COMMAND;	/* IOCTL CMD ID */
		fl_USB_tx_data_U8A[2] = clear_rx_buffer_ACK;	/* Command ID */
#endif
		((hfcpReq_t *) pbuf)->command = IOCTL_COMMAND;
		pRsp->ioctl_resp.com_id = clear_rx_buffer_ACK;
		(void)host_write((void *)pbuf,
				   hfcp_proto_cmd_len +
				   sizeof(pRsp->ioctl_resp));
		break;
	default:
#if 0
		fl_USB_tx_data_U8A[0] = buff[0];
		fl_USB_tx_data_U8A[1] = IOCTL_COMMAND;	//IOCTL CMD ID
		fl_USB_tx_data_U8A[2] = fl_IOCTL_CMD_U8;	//ACK for Get_config CMD
		fl_USB_tx_data_U8A[3] = ERR_INVALID_IOCTL_ID;
		fl_USB_tx_data_U8A[4] = buff[3];
#endif
		((hfcpReq_t *) pbuf)->command = IOCTL_COMMAND;
		pRsp->ioctl_resp.com_id = fl_IOCTL_CMD_U8;
		pRsp->ioctl_resp.status = ERR_INVALID_IOCTL_ID;
		pRsp->ioctl_resp.length = hfcpreq->u.ioctl_req.length;

		//(void) host_write((void *)fl_USB_tx_data_U8A,5);
		(void)host_write((void *)pbuf,
				   hfcp_proto_cmd_len +
				   sizeof(pRsp->ioctl_resp));

	}

	free(pbuf);
}

void process_CAN_FD_IOCTL_cmd(uint8_t * buff)
{
	hfcpReq_t *hfcpreq;
	hfcpResp_t *pRsp;
	uint32_t proto_id;
	void *pbuf;
	int hfcp_proto_cmd_len;

	uint8_t fl_IOCTL_CMD_U8 = 0, ch_id = 0;
	uint32_t fl_current_baudrate_U32 = 0;
	uint32_t fl_Protocol_ID_U8 = 0;
	uint8_t fl_loopback_status_U8, fl_SAM_status_U8, fl_SJW_status_U8;
	uint8_t fl_set_BR_ret_status_U8, fl_filt_stopAll_status;
	uint8_t fl_Q_clear_status;
	uint8_t fl_ps_chan_status = 0;
	uint8_t fl_connection_flag_U8;

	DBG("process_CAN_FD_IOCTL_cmd - Enter\n");

	hfcpreq = (hfcpReq_t *) buff;
	pbuf = malloc(USB_PKT_SIZE);
	if (pbuf == NULL) {
		DBG("malloc failed");
		return;
	}
	memset(pbuf, 0, sizeof(USB_PKT_SIZE));
	proto_id = hfcpreq->proto_id;
	hfcp_proto_cmd_len = sizeof(hfcpReq_t) - sizeof(hfcpreq->u);
	((hfcpReq_t *) pbuf)->proto_id = proto_id;
	((hfcpReq_t *) pbuf)->command = IOCTL_RESPONSE;
	pRsp = (hfcpResp_t *) ((uint8_t *) pbuf + hfcp_proto_cmd_len);
	fl_IOCTL_CMD_U8 = hfcpreq->u.ioctl_req.command_ID;	//IOCTL Command

	DBG("fl_IOCTL_CMD_U8=0x%x", fl_IOCTL_CMD_U8);

	if ((proto_id == CAN_FD_PROTOCOL_ID) ||
	    (proto_id == ISO15765_FD_PROTO_ID)) {
		ch_id = 1;
	} else if ((proto_id == CAN_FD_CH1_PROTO_ID) ||
		   (proto_id == ISO15765_FD_CH1_PROTO_ID)) {
		ch_id = 2;
	} else if (proto_id == CAN_FD_PS) {
		ch_id = CAN_FD_PS_chan;
	} else {
		//error
	}

	switch (fl_IOCTL_CMD_U8) {
	case Get_config:
		{
			DBG("CAN_FD_IOCTL -> Get_config - Enter");
			pRsp->ioctl_resp.com_id = Get_config_ACK;

			if (Get_data_rate == hfcpreq->u.ioctl_req.data) {
				DBG("Get_config - > Get_data_rate - Enter");
				fl_current_baudrate_U32 =
				    CAN_Mid_get_current_baudrate(ch_id);
				if (0xFFFFFFFF == fl_current_baudrate_U32) {
					pRsp->ioctl_resp.status = ERR_FAILED;
				} else {
					pRsp->ioctl_resp.status =
					    STATUS_NOERROR;
				}
				pRsp->ioctl_resp.length =
				    hfcpreq->u.ioctl_req.length;
				pRsp->ioctl_resp.data_rate = Get_data_rate_ACK;
				pRsp->ioctl_resp.u.baudrate =
				    fl_current_baudrate_U32;
			} else if (Get_loopback == hfcpreq->u.ioctl_req.data) {
				DBG("Get_config - > Get_loopback - Enter");
				fl_loopback_status_U8 =
				    LowLevel_CAN_get_loopback_status(ch_id);
				if (ERR_INVALID_CHANNEL_ID ==
				    fl_loopback_status_U8) {
					pRsp->ioctl_resp.status =
					    ERR_INVALID_CHANNEL_ID;
				} else {
					pRsp->ioctl_resp.status =
					    fl_loopback_status_U8;
				}
				pRsp->ioctl_resp.length =
				    hfcpreq->u.ioctl_req.length;
				pRsp->ioctl_resp.data_rate = Get_data_rate_ACK;
				pRsp->ioctl_resp.u.loopback_status =
				    fl_loopback_status_U8;
			} else if (Get_sample_point ==
				   hfcpreq->u.ioctl_req.data) {
				DBG("Get_config - > Get_sample_point - Enter");
				fl_SAM_status_U8 =
				    LowLevel_CAN_get_current_sampling_mode
				    (ch_id);
				if (ERR_INVALID_CHANNEL_ID == fl_SAM_status_U8) {
					pRsp->ioctl_resp.status =
					    ERR_INVALID_CHANNEL_ID;
				} else {
					pRsp->ioctl_resp.status =
					    fl_SAM_status_U8;
				}
				pRsp->ioctl_resp.length =
				    hfcpreq->u.ioctl_req.length;
				pRsp->ioctl_resp.data_rate =
				    Get_sample_point_ACK;
			} else if (Get_SJW == hfcpreq->u.ioctl_req.data) {
				DBG("Get_config - > Get_SJW - Enter");
				fl_SJW_status_U8 =
				    LowLevel_CAN_get_current_SJW(ch_id);
				if (ERR_INVALID_CHANNEL_ID == fl_SJW_status_U8) {
					pRsp->ioctl_resp.status =
					    ERR_INVALID_CHANNEL_ID;
				} else {
					pRsp->ioctl_resp.status =
					    fl_SJW_status_U8;
				}
				pRsp->ioctl_resp.length =
				    hfcpreq->u.ioctl_req.length;
				pRsp->ioctl_resp.data_rate = Get_SJW_ACK;
			} else if (Get_ps_chan == hfcpreq->u.ioctl_req.data) {
				//CAN_FD_PS_chan = hfcpreq->u.ioctl_req.un.CAN_FD_PS_chan;

				pRsp->ioctl_resp.status = STATUS_NOERROR;
				pRsp->ioctl_resp.length =
				    hfcpreq->u.ioctl_req.length;
				pRsp->ioctl_resp.data_rate = Get_ps_chan_ACK;
				pRsp->ioctl_resp.u.CAN_FD_PS_chan =
				    CAN_FD_PS_chan;
			} else if (Get_frame_length  == hfcpreq->u.ioctl_req.data) {
				//CAN_FD_PS_chan = hfcpreq->u.ioctl_req.un.CAN_FD_PS_chan;

				pRsp->ioctl_resp.status = STATUS_NOERROR;
				pRsp->ioctl_resp.length =
				    hfcpreq->u.ioctl_req.length;
				pRsp->ioctl_resp.data_rate = Get_frame_length_ACK;
				pRsp->ioctl_resp.u.CAN_FD_frame_length =
				    fd_frame_length;
			} else {
				DBG("Get_config - > Get_INVALID_IOCTL_ID - Enter");
				pRsp->ioctl_resp.status = ERR_INVALID_IOCTL_ID;
				pRsp->ioctl_resp.length =
				    hfcpreq->u.ioctl_req.length;
				pRsp->ioctl_resp.data_rate = fl_IOCTL_CMD_U8;
			}
		}
		(void)host_write((void *)pbuf,
				   hfcp_proto_cmd_len +
				   sizeof(pRsp->ioctl_resp));
		break;
	case Set_config:
		{
			DBG("CAN_FD_IOCTL -> Set_config - Enter");
			pRsp->ioctl_resp.com_id = Set_config_ACK;

			if (Set_data_rate == hfcpreq->u.ioctl_req.data) {
				DBG("Set_config - > Set_data_rate - Enter");
				fl_current_baudrate_U32 =
				    hfcpreq->u.ioctl_req.un.baudrate;
				CAN_Mid_DeInit((ch_id == 1) ? "can0" : "can1");
				CAN_Mid_Init(((ch_id ==
					       1) ? "can0" : "can1"),
					     fl_current_baudrate_U32);

				fl_set_BR_ret_status_U8 =
				    CAN_Mid_set_baudrate
				    (fl_current_baudrate_U32, ch_id);

				pRsp->ioctl_resp.status =
				    fl_set_BR_ret_status_U8;
				pRsp->ioctl_resp.length =
				    hfcpreq->u.ioctl_req.length;
				pRsp->ioctl_resp.data_rate = Set_data_rate_ACK;
				pRsp->ioctl_resp.u.baudrate =
				    fl_current_baudrate_U32;
			} else if (Set_loopback == hfcpreq->u.ioctl_req.data) {
				DBG("Set_config - > Set_loopback - Enter");
				if (hfcpreq->u.ioctl_req.un.loopback_status ==
				    1) {
					fl_loopback_status_U8 =
					    LowLevel_CAN_enable_loopback(ch_id);
				} else if (hfcpreq->u.ioctl_req.un.
					   loopback_status == 0) {
					fl_loopback_status_U8 =
					    LowLevel_CAN_disable_loopback
					    (ch_id);
				} else {
					fl_loopback_status_U8 =
					    ERR_INVALID_IOCTL_ID;
				}
				pRsp->ioctl_resp.status = fl_loopback_status_U8;
				pRsp->ioctl_resp.length =
				    hfcpreq->u.ioctl_req.length;
				pRsp->ioctl_resp.data_rate = Set_loopback_ACK;
			} else
			    if ((Set_sample_point == hfcpreq->u.ioctl_req.data)
				|| (Set_SJW == hfcpreq->u.ioctl_req.data)) {
				DBG("Set_config - > Set_sample_point_SJW - Enter");
				pRsp->ioctl_resp.status = ERR_NOT_SUPPORTED;
				pRsp->ioctl_resp.length =
				    hfcpreq->u.ioctl_req.length;
				pRsp->ioctl_resp.data_rate =
				    Set_sample_point_ACK;
			} else if (Set_frame_length ==
				   hfcpreq->u.ioctl_req.data) {
				fd_frame_length =
				    hfcpreq->u.ioctl_req.un.fd_frame_length;

				pRsp->ioctl_resp.status = STATUS_NOERROR;
				pRsp->ioctl_resp.length =
				    hfcpreq->u.ioctl_req.length;
				pRsp->ioctl_resp.data_rate =
				    Set_frame_length_ACK;
			} else if (Set_ps_chan == hfcpreq->u.ioctl_req.data) {
				CAN_FD_PS_chan =
				    hfcpreq->u.ioctl_req.un.CAN_FD_PS_chan;
				if (CAN_FD_PS_chan == CAN_CH1) {
					CAN_or_ISO = proto_id;
					CAN_FD_or_ISO15765_FD = proto_id;
					CAN1_Vehicle_STMinValue = 0;
					fl_ps_chan_status =
					    CAN_Mid_Init(CAN_CHA1,
							 ps_baud_rate);
				} else if (CAN_FD_PS_chan == CAN_CH2) {
					CAN1_or_ISO1 = proto_id;
					CANCH1_or_ISO15765CH1_J1939CH1 =
					    proto_id;
					CAN2_vehicle_STMinValue = 0;
					fl_ps_chan_status =
					    CAN_Mid_Init(CAN_CHA2,
							 ps_baud_rate);
				}
#if 1
				if ((ERR_INVALID_BAUDRATE != fl_ps_chan_status)
				    && (ERR_INVALID_CHANNEL_ID !=
					fl_ps_chan_status)) {
					DBG("Check success");
					if ((CAN_connect_flags &
					     CAN_ID_BOTH) == CAN_ID_BOTH) {
						fl_connection_flag_U8 =
						    RX_BOTH_STD_EXT_MSG;
					} else
					    if ((CAN_connect_flags &
						 CAN_29BIT_ID) ==
						CAN_29BIT_ID) {
						fl_connection_flag_U8 =
						    RX_ONLY_EXT_MSG;
					} else
					    if ((CAN_connect_flags &
						 CAN_29BIT_ID) == 0) {
						fl_connection_flag_U8 =
						    RX_ONLY_STD_MSG;
					} else {
						fl_ps_chan_status =
						    ERR_INVALID_FLAGS;
					}
					if (fl_ps_chan_status !=
					    ERR_INVALID_FLAGS) {
						fl_ps_chan_status =
						    CAN_Mid_Enable
						    (CAN_CH1,
						     fl_connection_flag_U8);
						//CAN_Mid_Init (char *can, uint32_t p_baudrate);
						if (fl_ps_chan_status ==
						    STATUS_NOERROR) {
							l_connected_channels++;
							if (CAN_FD_PS_chan ==
							    CAN_CH1) {
								l_connected_channel_1
								    = 1;
							} else
							    if (CAN_FD_PS_chan
								== CAN_CH2) {
								l_connected_channel_2
								    = 1;
							}
						}
					} else {
						/* Do nothing as fl_ps_chan_status shas been modified */
					}
				} else {
					/* Do nothing as fl_ps_chan_status has been modified */
				}
				if (fl_ps_chan_status == STATUS_NOERROR) {
					CAN_active = 1;
				}
#endif
				pRsp->ioctl_resp.status = fl_ps_chan_status;
				pRsp->ioctl_resp.length =
				    hfcpreq->u.ioctl_req.length;
				pRsp->ioctl_resp.data_rate = Set_ps_chan_ACK;
			} else if (Set_fd_datarate == hfcpreq->u.ioctl_req.data) {
				DBG("Set_config - > Set_INVALID_IOCTL_ID - Enter");
			} else {
				DBG("Set_config - > Set_INVALID_IOCTL_ID - Enter");
				pRsp->ioctl_resp.status = ERR_INVALID_IOCTL_ID;
				pRsp->ioctl_resp.length =
				    hfcpreq->u.ioctl_req.length;
				pRsp->ioctl_resp.data_rate = fl_IOCTL_CMD_U8;
			}
		}
		(void)host_write((void *)pbuf,
				   hfcp_proto_cmd_len +
				   sizeof(pRsp->ioctl_resp));
		break;
	case clear_all_msg_filters:
			DBG("CAN_FD_IOCTL -> clear_all_msg_filters - Enter");
			pRsp->ioctl_resp.com_id = clear_all_msg_filters_ACK;

			if ((proto_id == ISO15765_FD_PROTO_ID)
			    || (proto_id == CAN_FD_PROTOCOL_ID)) {
				fl_Protocol_ID_U8 = CAN_FD_PROTOCOL_ID;
			} else if ((proto_id == ISO15765_FD_CH1_PROTO_ID)
				   || (proto_id == CAN_FD_CH1_PROTO_ID)) {
				fl_Protocol_ID_U8 = CAN_FD_CH1_PROTO_ID;
			}
			fl_filt_stopAll_status =
			    J2534_ClearAllFilter(fl_Protocol_ID_U8);
			if (fl_filt_stopAll_status == J2534_NO_ERROR) {
				pRsp->ioctl_resp.status = STATUS_NOERROR;
			} else {
				pRsp->ioctl_resp.status = ERR_FAILED;
			}
		(void)host_write((void *)pbuf,
				   hfcp_proto_cmd_len +
				   sizeof(pRsp->ioctl_resp));
		break;
	case clear_all_periodic_msgs:
			DBG("CAN_FD_IOCTL -> clear_all_periodic_msgs - Enter");
			pRsp->ioctl_resp.com_id = clear_all_periodic_msgs_ACK;

			if ((proto_id == ISO15765_FD_PROTO_ID)
			    || (proto_id == CAN_FD_PROTOCOL_ID)) {
				fl_Protocol_ID_U8 = CAN_FD_PROTOCOL_ID;
			} else if ((proto_id == ISO15765_FD_CH1_PROTO_ID)
				   || (proto_id == CAN_FD_CH1_PROTO_ID)) {
				fl_Protocol_ID_U8 = CAN_FD_CH1_PROTO_ID;
			}
			fl_filt_stopAll_status = J2534_NO_ERROR;	//J2534_ClearAllPeriodic(fl_Protocol_ID_U8);
			if (fl_filt_stopAll_status == J2534_NO_ERROR) {
				pRsp->ioctl_resp.status = STATUS_NOERROR;
			} else {
				pRsp->ioctl_resp.status = ERR_FAILED;
			}
		(void)host_write((void *)pbuf,
				   hfcp_proto_cmd_len +
				   sizeof(pRsp->ioctl_resp));
		break;
	case clear_tx_buffer:
			DBG("CAN_FD_IOCTL -> clear_tx_buffer - Enter");
			pRsp->ioctl_resp.com_id = clear_tx_buffer_ACK;

			if ((proto_id == ISO15765_FD_PROTO_ID)
			    || (proto_id == CAN_FD_PROTOCOL_ID)) {
				fl_Q_clear_status = MID_PASS;
			} else if ((proto_id == ISO15765_FD_CH1_PROTO_ID)
				   || (proto_id == CAN_FD_CH1_PROTO_ID)) {
				fl_Q_clear_status = MID_PASS;
			}
			if (fl_Q_clear_status == MID_PASS) {
				pRsp->ioctl_resp.status = STATUS_NOERROR;	/* Staus No_ERROR */
			} else {
				pRsp->ioctl_resp.status = ERR_FAILED;	/* Failed due to Wrong Protocol ID */
			}
		(void)host_write((void *)pbuf,
				   hfcp_proto_cmd_len +
				   sizeof(pRsp->ioctl_resp));
		break;
	case clear_rx_buffer:
			DBG("CAN_FD_IOCTL -> clear_rx_buffer - Enter");
			pRsp->ioctl_resp.com_id = clear_rx_buffer_ACK;

			if ((proto_id == ISO15765_FD_PROTO_ID)
			    || (proto_id == CAN_FD_PROTOCOL_ID)) {
				fl_Q_clear_status = MID_PASS;
			} else if ((proto_id == ISO15765_FD_CH1_PROTO_ID)
				   || (proto_id == CAN_FD_CH1_PROTO_ID)) {
				fl_Q_clear_status = MID_PASS;
			}
			if (fl_Q_clear_status == MID_PASS) {
				pRsp->ioctl_resp.status = STATUS_NOERROR;	/* Staus No_ERROR */
			} else {
				pRsp->ioctl_resp.status = ERR_FAILED;	/* Failed due to Wrong Protocol ID */
			}
		(void)host_write((void *)pbuf,
				   hfcp_proto_cmd_len +
				   sizeof(pRsp->ioctl_resp));
		break;
	default:
			DBG("CAN_FD_IOCTL -> Get_config - Enter");
			pRsp->ioctl_resp.com_id = fl_IOCTL_CMD_U8;
			pRsp->ioctl_resp.status = ERR_INVALID_IOCTL_ID;
			pRsp->ioctl_resp.length = hfcpreq->u.ioctl_req.length;
		(void)host_write((void *)pbuf,
				   hfcp_proto_cmd_len +
				   sizeof(pRsp->ioctl_resp));
	}

	free(pbuf);
}



void process_KWP_command(uint8_t * buffer)
{
	hfcpReq_t * KWP_buffer = (hfcpReq_t*)buffer;	
	uint8_t fl_status_U8, fl_USB_tx_data_U8A[IN_BUFFER_SIZE], command = 0;
	PERIODIC_MSG j2534_periodic_msg;
	uint8_t periodic_msg_cmd = 0;
	J2534_filter_t 	fl_filter_config;			;
	uint8_t fl_FilterID, fl_Protocol_ID_U8;
	uint32_t loop_count = 0;
	J2534_stError_t fl_filt_config_status, fl_filt_stop_status;
	uint32_t current_time_stamp = 0;
	ISO9141_14230_Init_S fl_App_ISO9141_14230Init_S;
	ISO9141_14230_RETCODE fl_ISO9141_14230RetStatus;
	uint16_t fl_IdxLen;
	lv_t *plv;
	/*hfcpResp_kwp_t hfcpResp_kwp;
	hfcpResp_kwp_t * hfcpResp_kwp_buf = &hfcpResp_kwp;*/
	
	hfcpResp_kwp_t * hfcpResp_kwp_buf = (hfcpResp_kwp_t*)malloc(sizeof(hfcpResp_kwp_t));
	
	if (hfcpResp_kwp_buf == NULL) {
		DBG("malloc failed");
		return;
	}

	/* The Length has to retained with successive Calls */
	static uint16_t fl_KWPTX_LocalLen;

	command = KWP_buffer->command;
	hfcpResp_kwp_buf->resp_proto_id = KWP_buffer->proto_id ;

	switch (command) {
	case KWP_EnableComm:
		{
			#if 0
			memset(&fl_USB_tx_data_U8A, 0, IN_BUFFER_SIZE);
			
			fl_USB_tx_data_U8A[0] = buffer[0];
			fl_USB_tx_data_U8A[1] = buffer[1];
			fl_USB_tx_data_U8A[2] = buffer[2];
			fl_USB_tx_data_U8A[3] = buffer[3];
			
			fl_USB_tx_data_U8A[4] = KWP_EnableComm_ACK;
			#endif
			
			hfcpResp_kwp_buf->resp_command = KWP_EnableComm_ACK ;
			
			if ((KWP_buffer->proto_id == KWP_PROTOCOL_ID)  || (KWP_buffer->proto_id == ISO_9141_PROTO_ID)) {

				ISO_9141_OR_14230 = KWP_buffer->proto_id; // changed to uint32_t

				/* Determine the Initialization structure */
				fl_App_ISO9141_14230Init_S.ProtocolId =  KWP_buffer->proto_id ;
				fl_App_ISO9141_14230Init_S.Baudrate   =  KWP_buffer->u.kwp_enable.baudrate;
				fl_App_ISO9141_14230Init_S.Flags      =  KWP_buffer->u.kwp_enable.conn_flags;
				if (((KWP_buffer->u.kwp_enable.conn_flags & ISO9141_K_LINE_ONLY) == ISO9141_K_LINE_ONLY) ||
				    ((KWP_buffer->u.kwp_enable.conn_flags & ISO9141_NO_CHECKSUM) == ISO9141_NO_CHECKSUM) ||
				    (KWP_buffer->u.kwp_enable.conn_flags  == 0x0000000)) {
					/* Call the Driver Init function */
					ISO9141_14230_Init (&fl_App_ISO9141_14230Init_S);
					KWP_active = 1;
					
					hfcpResp_kwp_buf->un.status = STATUS_NOERROR;
				//	fl_USB_tx_data_U8A[5] = STATUS_NOERROR; // kwp_ch_indx
				} else {
				//	fl_USB_tx_data_U8A[5] =  ERR_INVALID_FLAGS; // kwp_ch_indx
					hfcpResp_kwp_buf->un.status = ERR_INVALID_FLAGS;
				}
			} else {
				/* ERROR: It should never enter here */
				hfcpResp_kwp_buf->un.status = ERR_INVALID_PROTOCOL_ID;  // kwp_ch_indx
				(void)host_write((void *) hfcpResp_kwp_buf, ((sizeof(hfcpResp_kwp_t)- sizeof(hfcpResp_kwp_buf->un)) + sizeof(hfcpResp_kwp_buf->un.status))); 
				break;
			}
			l_connected_channels++;
			l_connected_channel_Kline = 1;
			(void)host_write((void *) hfcpResp_kwp_buf, ((sizeof(hfcpResp_kwp_t)- sizeof(hfcpResp_kwp_buf->un)) + sizeof(hfcpResp_kwp_buf->un.status)));        
			break;
		}


	case KWP_DisableComm:
		{
			#if 0
			memset(&fl_USB_tx_data_U8A, 0, IN_BUFFER_SIZE);
			fl_USB_tx_data_U8A[0] = buffer[0];
			fl_USB_tx_data_U8A[1] = buffer[1];
			fl_USB_tx_data_U8A[2] = buffer[2];
			fl_USB_tx_data_U8A[3] = buffer[3];
			fl_USB_tx_data_U8A[4] = KWP_DisableComm_ACK;
			#endif
			//hfcpResp_kwp_buf->resp_proto_id = KWP_buffer->proto_id ;
			hfcpResp_kwp_buf->resp_command = KWP_DisableComm_ACK ;
			
			J2534_ClearAllFilter(KWP_PROTOCOL_ID);	//buffer[0]);
			suspend_pmsg(KWP_PROTOCOL_ID);	//buffer[0]); /* Suspend all periodic messages */
			ISO9141_14230_Reset();

			if ((KWP_buffer->proto_id == KWP_PROTOCOL_ID) || (KWP_buffer->proto_id == ISO_9141_PROTO_ID)) {
				ISO_9141_OR_14230 = 0;
				if (l_connected_channels) {
					l_connected_channels--;
				}
				l_connected_channel_Kline = 0;
				KWP_active = 0;
				//stop_time_stamp(timestamp_id[GARUDA_KWP_CH1]);
			} else {
				/* ERROR: It should never enter here */
				
				hfcpResp_kwp_buf->un.status = ERR_INVALID_PROTOCOL_ID;
				
				(void)host_write((void *) hfcpResp_kwp_buf, ((sizeof(hfcpResp_kwp_t) - sizeof(hfcpResp_kwp_buf->un)) + sizeof(hfcpResp_kwp_buf->un.status)));
				break;
			}
			hfcpResp_kwp_buf->un.status = STATUS_NOERROR;
			(void)host_write((void *) hfcpResp_kwp_buf, ((sizeof(hfcpResp_kwp_t) - sizeof(hfcpResp_kwp_buf->un)) + sizeof(hfcpResp_kwp_buf->un.status)));
			break;
		}


	case IOCTL_COMMAND:
		{
			process_KWP_IOCTL_cmd(buffer);
			break;
		}

	case KWP_Send_msg:
		{
			if ((KWP_buffer->proto_id  == KWP_PROTOCOL_ID) || (KWP_buffer->proto_id  == ISO_9141_PROTO_ID)) {
			
				/* Check the Segment Number if it is 0 or 1 copy the Flag and Length Details */
				if (KWP_buffer->u.kwp_sndmsg.seg_num == SEG_NUM_ZERO) {
				
					/*l_App_ISO9141_14230TxMsg_S.Length = ((uint16_t) buffer[3] | (uint16_t) buffer[4] << 8); */
					
					ISO9141_14230_TxMsg_S_Buffer.Length = KWP_buffer->u.kwp_sndmsg.tx_msg_len;  

					
					
						/* If Length is 0 then dont copy data and Flags */
						if (ISO9141_14230_TxMsg_S_Buffer.Length != 0) {
						
							//l_App_ISO9141_14230TxMsg_S.Flags = ((uint32_t)buffer[5] | (uint32_t)buffer[6] << 8 | (uint32_t)buffer[7] << 16 | (uint32_t)buffer[8] << 24);

							ISO9141_14230_TxMsg_S_Buffer.Flags = KWP_buffer->u.kwp_sndmsg.tx_msg_flags;
							
							/*for (fl_IdxLen = 0; fl_IdxLen < l_App_ISO9141_14230TxMsg_S.Length; fl_IdxLen++) {
								l_App_ISO9141_14230TxMsg_S.Data[fl_IdxLen] = buffer[9 + fl_IdxLen];
							}*/
							for (fl_IdxLen = 0; fl_IdxLen < ISO9141_14230_TxMsg_S_Buffer.Length;fl_IdxLen++) {
								ISO9141_14230_TxMsg_S_Buffer.Data[fl_IdxLen] = buffer[12 + fl_IdxLen];
							}
							
						} else {
							/* Do Nothing */
						}
						
						/* MArk the Flag = NO Segmented Transfer */
					//	l_KWPTX_SegTrnsfr = 0;
						
						/* Call the Write Message function */
						fl_ISO9141_14230RetStatus = ISO9141_14230_WriteMsg();

						/* Determine the Response */
					/*       memset(&fl_USB_tx_data_U8A, 0, IN_BUFFER_SIZE);
						
						    
					         fl_USB_tx_data_U8A[0] = buffer[0];
						 fl_USB_tx_data_U8A[1] = buffer[1];
						 fl_USB_tx_data_U8A[2] = buffer[2];
						 fl_USB_tx_data_U8A[3] = buffer[3];
						 fl_USB_tx_data_U8A[4] = KWP_Send_msg_ACK;
						 fl_USB_tx_data_U8A[5] = KWP_buffer->u.kwp_sndmsg.seg_num; */
					
					
						hfcpResp_kwp_buf->resp_command = KWP_Send_msg_ACK;
						hfcpResp_kwp_buf->un.kwp_sendmsg_resp.seg_num = KWP_buffer->u.kwp_sndmsg.seg_num;
						 
						/* Update time stamp on USB TX Frame */ // Doubt because timestamp updated by local structure at write msg 
						//current_time_stamp = ISO9141_14230_TxMsg_S_Buffer.Timestamp; //Need to check
						
						#if 0
						fl_USB_tx_data_U8A[7] = (uint8_t) ((current_time_stamp) & 0xFF);
						fl_USB_tx_data_U8A[8] = (uint8_t) ((current_time_stamp >> 8) & 0xFF);
						fl_USB_tx_data_U8A[9] = (uint8_t) ((current_time_stamp >> 16) & 0xFF);
						fl_USB_tx_data_U8A[10] = (uint8_t) ((current_time_stamp >> 24) & 0xFF);
						#endif
						
						hfcpResp_kwp_buf->un.kwp_sendmsg_resp.timestamp = current_time_stamp;

						/* Update the Error Code based on the return */
						/*if (fl_ISO9141_14230RetStatus == ISO9141_14230_TXQ_FULL) {
							fl_USB_tx_data_U8A[6] = ERR_BUFFER_FULL;	
						} 
						else {
							fl_USB_tx_data_U8A[6] = STATUS_NOERROR;
						}*/
						hfcpResp_kwp_buf->un.kwp_sendmsg_resp.status = STATUS_NOERROR;
						//(void)host_write((void *)fl_USB_tx_data_U8A, 11); // size needs to check 
						(void)host_write((void *) hfcpResp_kwp_buf, ((sizeof(hfcpResp_kwp_t) - sizeof(hfcpResp_kwp_buf->un)) + sizeof(hfcpResp_kwp_buf->un.kwp_sendmsg_resp)));
			
				}
			
				else {
					/*Do nothing*/
				}
			} else {
				/* ERROR: It should never enter here */
	
				#if 0
				fl_USB_tx_data_U8A[0] = buffer[0];
				fl_USB_tx_data_U8A[1] = buffer[1];
				fl_USB_tx_data_U8A[2] = buffer[2];
				fl_USB_tx_data_U8A[3] = buffer[3];
				fl_USB_tx_data_U8A[4] = KWP_Send_msg_ACK;
				fl_USB_tx_data_U8A[5] = KWP_buffer->u.kwp_sndmsg.seg_num;
				fl_USB_tx_data_U8A[6] = ERR_INVALID_PROTOCOL_ID;
				#endif 
				
				hfcpResp_kwp_buf->resp_command = KWP_Send_msg_ACK;
				hfcpResp_kwp_buf->un.kwp_sendmsg_resp.seg_num = KWP_buffer->u.kwp_sndmsg.seg_num;
				hfcpResp_kwp_buf->un.kwp_sendmsg_resp.status = ERR_INVALID_PROTOCOL_ID;
				
				//(void)host_write((void *)fl_USB_tx_data_U8A, 7);
				(void)host_write((void *) hfcpResp_kwp_buf, ((sizeof(hfcpResp_kwp_t) - sizeof(hfcpResp_kwp_buf->un)) + sizeof(hfcpResp_kwp_buf->un.kwp_sendmsg_resp)));
			
			}
			break;
		}

	case Start_msg_filter:
		if ((KWP_buffer->proto_id == KWP_PROTOCOL_ID) || (KWP_buffer->proto_id == ISO_9141_PROTO_ID)) {
			fl_Protocol_ID_U8 = KWP_PROTOCOL_ID;
		}

		/**< Changed for differentiating 14230 and 9141 protocol */
		fl_filter_config.Protocol_ID = fl_Protocol_ID_U8;	/* buffer[0]; */
		fl_filter_config.filterType = (J2534_filterType_t)KWP_buffer->u.kwp_msg_filter.filtertype;
		plv = (lv_t*) (KWP_buffer->u.kwp_msg_filter.buf);
		fl_filter_config.MaskLen = plv->len;
		
		for (loop_count = 0; loop_count < plv->len; loop_count++) {
			fl_filter_config.maskMsg[loop_count] = plv->buf[loop_count];
		}
		
		plv = (lv_t*) (KWP_buffer->u.kwp_msg_filter.buf) + sizeof(plv->len) + plv->len;
		fl_filter_config.PatternLen = plv->len;
		
		for (loop_count = 0; loop_count < plv->len; loop_count++) {
			fl_filter_config.patternMsg[loop_count] = plv->buf[loop_count];
		}
		
		fl_filt_config_status = J2534_ConfigFilter(&fl_filter_config, &fl_FilterID);
		
		if (fl_filt_config_status != J2534_NO_ERROR) {
			if (fl_filt_config_status == J2534_FLT_NOT_FREE) {
				fl_status_U8 = ERR_EXCEEDED_LIMIT;
			} else {
				fl_status_U8 = ERR_FAILED;
			}
		/*	memset(&fl_USB_tx_data_U8A, 0, IN_BUFFER_SIZE);
			fl_USB_tx_data_U8A[0] = buffer[0];
			fl_USB_tx_data_U8A[1] = buffer[1];
			fl_USB_tx_data_U8A[2] = buffer[2];
			fl_USB_tx_data_U8A[3] = buffer[3];
			fl_USB_tx_data_U8A[4] = Start_msg_filter_ACK;
			fl_USB_tx_data_U8A[5] = fl_status_U8;
			fl_USB_tx_data_U8A[6] = fl_FilterID;
			//(void)Garuda_Tx_data_on_USB(&fl_USB_tx_data_U8A[0],5,DONT_RELEASE);
			(void)host_write((void *)fl_USB_tx_data_U8A, 7);
		*/
			hfcpResp_kwp_buf->resp_command = Start_msg_filter_ACK;
			hfcpResp_kwp_buf->un.kwp_msg_filter_resp.status = fl_status_U8;
			hfcpResp_kwp_buf->un.kwp_msg_filter_resp.filterid = fl_FilterID;
			
			(void)host_write((void *) hfcpResp_kwp_buf, ((sizeof(hfcpResp_kwp_t) - sizeof(hfcpResp_kwp_buf->un)) + sizeof(hfcpResp_kwp_buf->un.kwp_msg_filter_resp)));	
		
		
		} else {
		/*	memset(&fl_USB_tx_data_U8A, 0, IN_BUFFER_SIZE);
			fl_USB_tx_data_U8A[0] = buffer[0];
			fl_USB_tx_data_U8A[1] = buffer[1];
			fl_USB_tx_data_U8A[2] = buffer[2];
			fl_USB_tx_data_U8A[3] = buffer[3];
			fl_USB_tx_data_U8A[4] = Start_msg_filter_ACK;
			fl_USB_tx_data_U8A[5] = STATUS_NOERROR;
			fl_USB_tx_data_U8A[6] = fl_FilterID;
			//(void)Garuda_Tx_data_on_USB(&fl_USB_tx_data_U8A[0],5,DONT_RELEASE);
			(void)host_write((void *) fl_USB_tx_data_U8A, 7); */
			
			hfcpResp_kwp_buf->resp_command = Start_msg_filter_ACK;
			hfcpResp_kwp_buf->un.kwp_msg_filter_resp.status = STATUS_NOERROR;
			hfcpResp_kwp_buf->un.kwp_msg_filter_resp.filterid = fl_FilterID;
			
			(void)host_write((void *) hfcpResp_kwp_buf, ((sizeof(hfcpResp_kwp_t) - sizeof(hfcpResp_kwp_buf->un)) + sizeof(hfcpResp_kwp_buf->un.kwp_msg_filter_resp)));	
		
		
		}
		break;
		

	case Stop_msg_filter:
		if ((KWP_buffer->proto_id == KWP_PROTOCOL_ID) || (KWP_buffer->proto_id == ISO_9141_PROTO_ID)) {
			fl_Protocol_ID_U8 = KWP_PROTOCOL_ID;
		}

		/* fl_Protocol_ID_U8 = buffer[0]; */

		/**< Commented to have a common Id for 9141 and 14230 */
		fl_FilterID = KWP_buffer->u.kwp_stpmsg_filter.filterid;
		fl_filt_stop_status = J2534_ClearFilter(fl_FilterID, fl_Protocol_ID_U8);
		if (fl_filt_stop_status != J2534_NO_ERROR) {
			if (fl_filt_stop_status == J2534_INVLD_FLTID) {
				fl_status_U8 = ERR_INVALID_FILTER_ID;
			} else {
				fl_status_U8 = ERR_FAILED;
			}
			/*memset(&fl_USB_tx_data_U8A, 0, IN_BUFFER_SIZE);
			fl_USB_tx_data_U8A[0] = buffer[0];
			fl_USB_tx_data_U8A[1] = buffer[1];
			fl_USB_tx_data_U8A[2] = buffer[2];
			fl_USB_tx_data_U8A[3] = buffer[3];
			fl_USB_tx_data_U8A[4] = Stop_msg_filter_ACK;
			fl_USB_tx_data_U8A[5] = fl_status_U8;
			fl_USB_tx_data_U8A[6] = fl_FilterID;
			//(void)Garuda_Tx_data_on_USB(&fl_USB_tx_data_U8A[0],4,DONT_RELEASE);
			(void)host_write((void *)fl_USB_tx_data_U8A, 7);*/
			
			hfcpResp_kwp_buf->resp_command = Stop_msg_filter_ACK;
			hfcpResp_kwp_buf->un.kwp_stpmsg_filter_resp.status = fl_status_U8;
			hfcpResp_kwp_buf->un.kwp_stpmsg_filter_resp.filterid = fl_FilterID;
			
			(void)host_write((void *) hfcpResp_kwp_buf, ((sizeof(hfcpResp_kwp_t) - sizeof(hfcpResp_kwp_buf->un)) + sizeof(hfcpResp_kwp_buf->un.kwp_stpmsg_filter_resp)));	
		
			
		} else {
			/*memset(&fl_USB_tx_data_U8A, 0, IN_BUFFER_SIZE);
			fl_USB_tx_data_U8A[0] = buffer[0];
			fl_USB_tx_data_U8A[1] = buffer[1];
			fl_USB_tx_data_U8A[2] = buffer[2];
			fl_USB_tx_data_U8A[3] = buffer[3];
			fl_USB_tx_data_U8A[4] = Stop_msg_filter_ACK;
			fl_USB_tx_data_U8A[5] = STATUS_NOERROR;
			fl_USB_tx_data_U8A[6] = fl_FilterID;
			//(void)Garuda_Tx_data_on_USB(&fl_USB_tx_data_U8A[0],4,DONT_RELEASE);
			(void)host_write((void *)fl_USB_tx_data_U8A, 7);*/
			
			hfcpResp_kwp_buf->resp_command = Stop_msg_filter_ACK;
			hfcpResp_kwp_buf->un.kwp_stpmsg_filter_resp.status = STATUS_NOERROR;
			hfcpResp_kwp_buf->un.kwp_stpmsg_filter_resp.filterid = fl_FilterID;
			
			(void)host_write((void *) hfcpResp_kwp_buf, ((sizeof(hfcpResp_kwp_t) - sizeof(hfcpResp_kwp_buf->un)) + sizeof(hfcpResp_kwp_buf->un.kwp_stpmsg_filter_resp)));	
			
		}
		break;

	case handle_periodic_msg:
		if ((KWP_buffer->proto_id == KWP_PROTOCOL_ID) || (KWP_buffer->proto_id == ISO_9141_PROTO_ID)) {
			fl_Protocol_ID_U8 = KWP_PROTOCOL_ID;
		}

		periodic_msg_cmd = KWP_buffer->u.kwp_periodic_msg.periodic_command;
		j2534_periodic_msg.protocol_id = fl_Protocol_ID_U8;	/* buffer[0]; */
		    
		j2534_periodic_msg.periodicity = KWP_buffer->u.kwp_periodic_msg.periodicity;
		j2534_periodic_msg.prmsg_id = KWP_buffer->u.kwp_periodic_msg.msgid;
		j2534_periodic_msg.proto_msg.tx_flags = KWP_buffer->u.kwp_periodic_msg.tx_flags;

		if ((periodic_msg_cmd == START_NEW_PERIODIC_MSG_TXN) || (periodic_msg_cmd == UPDATE_DATA_TO_MSG_ID)) {
			j2534_periodic_msg.proto_msg.length = KWP_buffer->u.kwp_periodic_msg.msglen;
		} else {
			j2534_periodic_msg.proto_msg.length = 0;
		}

		for (loop_count = 0; loop_count < j2534_periodic_msg.proto_msg.length; loop_count++) {
			j2534_periodic_msg.proto_msg.data[loop_count] = KWP_buffer->u.kwp_periodic_msg.buf[loop_count];
		}

		if (PERIODIC_SUCCESS == PERIODIC_msg_cmd(&j2534_periodic_msg, periodic_msg_cmd)) {
			fl_status_U8 = STATUS_NOERROR;
		} else {
			fl_status_U8 = ERR_FAILED;
		}
		/* Send Acknowledgement to J2534 DLL */
		/*memset(&fl_USB_tx_data_U8A, 0, IN_BUFFER_SIZE);
		fl_USB_tx_data_U8A[0] = buffer[0];
		fl_USB_tx_data_U8A[1] = buffer[1];
		fl_USB_tx_data_U8A[2] = buffer[2];
		fl_USB_tx_data_U8A[3] = buffer[3];
		fl_USB_tx_data_U8A[4] = handle_periodic_msg_ACK;
		fl_USB_tx_data_U8A[5] = fl_status_U8;
		fl_USB_tx_data_U8A[6] = j2534_periodic_msg.prmsg_id;
		//(void)Garuda_Tx_data_on_USB(&fl_USB_tx_data_U8A[0],4,DONT_RELEASE);
		(void)host_write((void *)fl_USB_tx_data_U8A, 7);*/
		
		hfcpResp_kwp_buf->resp_command = handle_periodic_msg_ACK;
		hfcpResp_kwp_buf->un.kwp_periodic_msg_resp.status = fl_status_U8;
		hfcpResp_kwp_buf->un.kwp_periodic_msg_resp.per_msg_id = j2534_periodic_msg.prmsg_id;
			
		(void)host_write((void *) hfcpResp_kwp_buf, ((sizeof(hfcpResp_kwp_t) - sizeof(hfcpResp_kwp_buf->un)) + sizeof(hfcpResp_kwp_buf->un.kwp_periodic_msg_resp)));	
			
		
		break;

	default:
		break;
	}

	return;
}


static void process_KWP_IOCTL_cmd(uint8_t * buffer)
{
	hfcpReq_t *KWP_buffer = (hfcpReq_t*)buffer;  //ch
	uint32_t fl_KWP_par_val_U32;
	uint8_t fl_USB_tx_data_U8A[IN_BUFFER_SIZE];
	J2534_stError_t fl_filt_stopAll_status;
	ISO9141_14230_Cmd_S fl_App_ISO9141_14230Cmd_S;
	ISO9141_14230_RETCODE fl_ISO9141_14230RetStatus = NO_ERROR;
	
	
	/*hfcpResp_kwp_t hfcpResp_kwp;
	hfcpResp_kwp_t * hfcpResp_kwp_buf = &hfcpResp_kwp;*/
	
	hfcpResp_kwp_t * hfcpResp_kwp_buf = (hfcpResp_kwp_t*)malloc(sizeof(hfcpResp_kwp_t));
	
	if (hfcpResp_kwp_buf == NULL) {
		DBG("malloc failed");
		return;
	}


	//uint8_t  fl_channel_no_U8;  

	if ((KWP_buffer->proto_id == KWP_PROTOCOL_ID) || (KWP_buffer->proto_id == ISO_9141_PROTO_ID)) {  //ch
		
		//fl_channel_no_U8 = 1;
	} else {
		//fl_channel_no_U8 = 2;
	}
	
	#if 0
	memset(&fl_USB_tx_data_U8A, 0, IN_BUFFER_SIZE);

	fl_USB_tx_data_U8A[0] = buffer[0];   //ch
	fl_USB_tx_data_U8A[1] = buffer[1];
	fl_USB_tx_data_U8A[2] = buffer[2];
	fl_USB_tx_data_U8A[3] = buffer[3];
	fl_USB_tx_data_U8A[4] = IOCTL_COMMAND;	/* IOCTL CMD ID */
	#endif 
	
	hfcpResp_kwp_buf->resp_proto_id = KWP_buffer->proto_id ;
	hfcpResp_kwp_buf->resp_command = IOCTL_COMMAND ;
	
	/* Determine the Command Structure */
	/* Get the Ioctl ID */
	fl_App_ISO9141_14230Cmd_S.IOCtlId = KWP_buffer->u.kwp_ioctl_req.command_ID;  //ch  
	/*Length data ( Only for Fivebaud and Fast Init) */
	fl_App_ISO9141_14230Cmd_S.Length = KWP_buffer->u.kwp_ioctl_req.length;  //ch

	/* Clear All Periodic Messages Command */
	if (clear_all_periodic_msgs == fl_App_ISO9141_14230Cmd_S.IOCtlId) {
		//fl_USB_tx_data_U8A[5] = clear_all_periodic_msgs_ACK; //ch
		hfcpResp_kwp_buf->un.kwp_ioctl_resp.ioctl_ack = clear_all_periodic_msgs_ACK;
		suspend_pmsg(KWP_PROTOCOL_ID);	// buffer[0]);
		/* Suspend all periodic messages */
		hfcpResp_kwp_buf->un.kwp_ioctl_resp.status = STATUS_NOERROR;	/* Error status */   //ch
		//(void)Garuda_Tx_data_on_USB(&fl_USB_tx_data_U8A[0],4,DONT_RELEASE);
		(void)host_write((void *)hfcpResp_kwp_buf, ((sizeof(hfcpResp_kwp_t) - sizeof(hfcpResp_kwp_buf->un)) + sizeof(hfcpResp_kwp_buf->un.kwp_ioctl_resp)));
	}
	/* Clear All Message Filters Command */
	else if (clear_all_msg_filters == fl_App_ISO9141_14230Cmd_S.IOCtlId) {

		hfcpResp_kwp_buf->un.kwp_ioctl_resp.ioctl_ack = clear_all_msg_filters_ACK;	/* ACK for Clear All Filters */  //ch
		fl_filt_stopAll_status = J2534_ClearAllFilter(KWP_PROTOCOL_ID);	//buffer[0]);
		if (fl_filt_stopAll_status == J2534_NO_ERROR) {
			hfcpResp_kwp_buf->un.kwp_ioctl_resp.status = STATUS_NOERROR;	/* Status No_ERROR */    //ch
		} else {
			hfcpResp_kwp_buf->un.kwp_ioctl_resp.status = ERR_FAILED;	/* Failed due to Wrong Protocol ID */  //ch
		}
		//(void)Garuda_Tx_data_on_USB(&fl_USB_tx_data_U8A[0],4,DONT_RELEASE);
		(void)host_write((void *)hfcpResp_kwp_buf, ((sizeof(hfcpResp_kwp_t) - sizeof(hfcpResp_kwp_buf->un)) + sizeof(hfcpResp_kwp_buf->un.kwp_ioctl_resp)));
	} else {
		/* Determine if the IOCtl is GET_CONFIG or SET_CONFIG to check for param id and param val */
		if ((fl_App_ISO9141_14230Cmd_S.IOCtlId == GET_CONFIG)  || (fl_App_ISO9141_14230Cmd_S.IOCtlId == SET_CONFIG)) {
			/* Parameter ID for Get Config and Setconfig */
			fl_App_ISO9141_14230Cmd_S.ParamId = KWP_buffer->u.kwp_ioctl_req.param_ID; //ch
			/* Copy the Parameter Value */
			if (fl_App_ISO9141_14230Cmd_S.IOCtlId == SET_CONFIG) {
			
				fl_KWP_par_val_U32 = KWP_buffer->u.kwp_ioctl_req.data;        //ch
			}
		
			fl_App_ISO9141_14230Cmd_S.pData = &fl_KWP_par_val_U32;
		}
		/* If IOCtl has no param id and value then assign address of data */
		else {
			//fl_App_ISO9141_14230Cmd_S.pData = &buffer[4];
		}

		/* Call the Driver Command function */
		fl_ISO9141_14230RetStatus =  ISO9141_14230_Command(&fl_App_ISO9141_14230Cmd_S);

		/* Determine the response */
		hfcpResp_kwp_buf->un.kwp_ioctl_resp.ioctl_ack = fl_App_ISO9141_14230Cmd_S.IOCtlId;  //ch
		hfcpResp_kwp_buf->un.kwp_ioctl_resp.length = fl_App_ISO9141_14230Cmd_S.Length;   //ch

		/* For Get / Set Config copy the Parameter value and Id to the Response frame */
		if ((fl_App_ISO9141_14230Cmd_S.IOCtlId == GET_CONFIG) || (fl_App_ISO9141_14230Cmd_S.IOCtlId == SET_CONFIG)) {
		  
		
			hfcpResp_kwp_buf->un.kwp_ioctl_resp.paramId = fl_App_ISO9141_14230Cmd_S.ParamId;                      //ch
		
			if (fl_App_ISO9141_14230Cmd_S.IOCtlId == GET_CONFIG) {
			
				hfcpResp_kwp_buf->un.kwp_ioctl_resp.data = fl_KWP_par_val_U32 ;
				#if 0
				fl_USB_tx_data_U8A[9]  = (uint8_t) ((fl_KWP_par_val_U32) & 0xFF);       //ch
				fl_USB_tx_data_U8A[10] = (uint8_t) ((fl_KWP_par_val_U32 >> 8) & 0xFF);  //ch
				fl_USB_tx_data_U8A[11] = (uint8_t) ((fl_KWP_par_val_U32 >>16) & 0xFF);  //ch
				fl_USB_tx_data_U8A[12] = (uint8_t) ((fl_KWP_par_val_U32 >>24) & 0xFF);  //ch
				#endif
			}
		}

		/* Update the Error Code based on the return */
		if (fl_ISO9141_14230RetStatus == NO_ERROR) {
			hfcpResp_kwp_buf->un.kwp_ioctl_resp.status = STATUS_NOERROR;                    //ch
		} else if (fl_ISO9141_14230RetStatus == INVALID_COMMAND) {
			hfcpResp_kwp_buf->un.kwp_ioctl_resp.status = ERR_INVALID_IOCTL_ID;              //ch
		} else if (fl_ISO9141_14230RetStatus == INVALID_PARAMETERID) {
			hfcpResp_kwp_buf->un.kwp_ioctl_resp.status = ERR_FAILED;                        //ch
		} else {
			/* Do nothing */
		}

		/* Send the Response only if its not FAST INIT ot FIVE BAUD INIT */
		if ((fl_App_ISO9141_14230Cmd_S.IOCtlId != FAST_INIT) && (fl_App_ISO9141_14230Cmd_S.IOCtlId != FIVE_BAUD_INIT)) {
			//(void)Garuda_Tx_data_on_USB(&fl_USB_tx_data_U8A[0],10,DONT_RELEASE);
			(void)host_write((void *)hfcpResp_kwp_buf, ((sizeof(hfcpResp_kwp_t) - sizeof(hfcpResp_kwp_buf->un)) + sizeof(hfcpResp_kwp_buf->un.kwp_ioctl_resp)));
		}
	}
}


void AdressClaimHandler(void)
{
	return;
}

int get_connected_channels(void)
{
	return (l_connected_channel_1 || l_connected_channel_2
		|| l_connected_channel_J1939_1
		|| l_connected_channel_J1939_2 || l_connected_channel_Kline);
}

void clear_connected_channels(void)
{
	return;
}

uint8_t get_CAN_or_ISO(void)
{
	return 0;
}

int get_CAN_or_ISO15756_or_J1939(void)
{
	return CAN_or_ISO15765_J1939;
}

int get_CAN_FD_or_ISO15756_FD(void)
{
	return CAN_FD_or_ISO15765_FD;
}

uint8_t get_CAN1_or_ISO1(void)
{
	return 0;
}

int get_CANCH1_or_ISO15756CH1_or_J1939CH1(void)
{
	return CANCH1_or_ISO15765CH1_J1939CH1;
}

int get_CAN_FDCH1_or_ISO15765_FDCH1(void)
{
	return CAN_FDCH1_or_ISO15765_FDCH1;
}

void hfcp_disable_CAN(void)
{
	return;
}

void HFCP_pauseTask(void)
{
	return;
}

void HFCP_startTask(void)
{
	return;
}

void HFCP_Task_Reset(void)
{
	return;
}

void hfcp_disable_CAN_CH1(void)
{
	return;
}

void suspend_J1939_addressClaim(void)
{
	return;
}

void J1939_AddressClaim_Global_Response(uint8_t * p_pJ2534Msg,
					uint16_t p_MsgLen,
					GARUDA_ChannelNum_t GarudaChannel)
{
	uint8_t loop_cnt;
	struct can_frame frame;
	Mid_API_Status_t fl_Q_Status;

	memset((void *)&frame, 0, sizeof(frame));
	frame.can_id = frame.can_id | 0x18 << 24;
	frame.can_id = frame.can_id | PDU_FORMAT_ADDRESS_CLAIM << 16;
	frame.can_id = frame.can_id | GLOBAL_ADDRESS << 8;
	frame.can_id = frame.can_id | device_claimed_Address << 0;

	frame.can_dlc = p_MsgLen - 4;

	for (loop_cnt = 0; loop_cnt < frame.can_dlc; loop_cnt++) {
		frame.data[loop_cnt] = AdressClaim.nameIdentifier[loop_cnt];
	}

	/* Add To Queue */
	if (GARUDA_J1939_CH1 == GarudaChannel)	//GARUDA_J1939_CH1
	{
		fl_Q_Status = CAN_Mid_Transmit(CAN_CH1, &frame);
	} else if (GARUDA_J1939_CH2 == GarudaChannel) {
		fl_Q_Status = CAN_Mid_Transmit(CAN_CH2, &frame);
	} else {
		/* Do Nothing */
	}
	if (fl_Q_Status == MID_FAIL) {
		/* Queue is Full -- Not Decided the Action next */
	}
}

void process_J1939_command(int *buffer)
{
	return;
}

int get_CAN_CH1_status(void)
{
	return 0;
}

int get_CAN_status(void)
{
	return 0;
}

int get_J1708_status(void)
{
	return 0;
}

int get_ISO9141_or_14230(void)
{
	return 0;
}

void DisconnectAllChanels(void)
{
	return;
}

int get_KWP_status(void)
{
	return 0;
}

/*
 * ------------------------------------------
 * | proto_id | command  | command payload  |
 * | (1 byte) | (1 byte) | (17 bytes)       |
 * ------------------------------------------
 *
 */
int process_doip_command(hfcpReq_t * hfcp)
{
	int ret, len;
	hfcpResp_t *resp;

	DBG("Received DoIP cmd %d\n", hfcp->command);

	len = sizeof(hfcp->proto_id) + sizeof(hfcp->command);
	resp = (hfcpResp_t *) ((uint8_t *) hfcp + len);

	switch (hfcp->command) {
	case DOIP_CLI_CMD_START_SESSION:
		resp->status =
		    doip_cli_cmd_start_session(&hfcp->u.start_session);
		break;
	case DOIP_CLI_CMD_ADD_DOIP_SERVER:
		resp->status = doip_cli_add_server(&hfcp->u.doip_sadd);
		break;
	case DOIP_CLI_CMD_ADD_NODE:
		resp->status = doip_cli_add_node(&hfcp->u.doip_addnode);
		break;
	case DOIP_CLI_CMD_SEND_DIAG_MSG:
		resp->status = doip_cli_cmd_send_diag_msg(&hfcp->u.diagmsg);
		break;
	case DOIP_CLI_CMD_REMOVE_DOIP_SERVER:
		resp->status = doip_cli_del_server(&hfcp->u.doip_sdel);
		break;
	case DOIP_CLI_CMD_DEL_NODE:
		resp->status = doip_cli_del_node(&hfcp->u.doip_delnode);
		break;
	case DOIP_IOCTL_HANDLING:
		resp->status = doip_ioctl_handling(&hfcp->u.doipioctl);
		break;
	case DOIP_CLIENT_DETAILS:      
		resp->status = doip_client_details(&hfcp->u.doipclient);
	        break;
	default:
		DBG("DoIP cmd - unhandled");
		resp->status = ERR_NOT_SUPPORTED;
		break;
	}

	len += sizeof(resp->status);
	ret = host_write((void *)hfcp, len);
	if (ret <= 0) {
		DBG("USB write failed: %s", strerror(errno));
	}

	return ret;
}

int Firmware_Update_command(uint8_t *buff, uint8_t len)
{
	uint8_t segment = 0;
	FirmwareUpgrade_t *Firmware = &buffer_t.f_buffer;
	size_t result;
        
	FirmwareHost_t *firmwaredetails;
	firmwaredetails = (FirmwareHost_t *)buff;

	const char *dest_path = "/tmp/g3update-M.m.n.tgz";
	static FILE *file = NULL;
	
	if (file == NULL) {
		file = fopen(dest_path, "ab");
		if (file == NULL) {
			perror("ERROR opening file");
			return EXIT_FAILURE;
		}
	}
  
	DBG("segment %d\n",firmwaredetails->seg_number);
	DBG("length %d\n", firmwaredetails->length);
	segment = firmwaredetails->seg_number;
	if (segment == 1) {
		buffer_t.pkt_number++;
		Firmware->length = firmwaredetails->length;
		Firmware->pkt_number = firmwaredetails->pkt_number;
		DBG("pkt_number %d\n", Firmware->pkt_number);
		if (Firmware->pkt_number == 1) {
			buffer_t.pkt_number = 1;
			buffer_t.is_ready_to_write = 0;
		}
		if (Firmware->length != 256) { //256

		}
	}

	if ((Firmware->length - Firmware->index) > 55) { // 55
		DBG("length %d\n", Firmware->length);
		DBG("index %d\n", Firmware->index);
		DBG("index %d\n", firmwaredetails->index);
		memcpy(&Firmware->buffer[Firmware->index],&firmwaredetails->index[0],55);//55
		Firmware->index += 55;//55;
		len -= 55; //55;
		DBG("length %d\n", Firmware->length);
		DBG("index %d\n", Firmware->index);
		DBG("loop %d\n",(Firmware->length - Firmware->index));
	} else {
		memcpy(&Firmware->buffer[Firmware->index], &firmwaredetails->index[0],
				Firmware->length - Firmware->index);
		len -= (Firmware->length - Firmware->index);
		Firmware->index += Firmware->length;
		buffer_t.is_ready_to_write = 1;
	}

	if (buffer_t.is_ready_to_write) {
		buffer_t.is_ready_to_write = 0;
		if (Firmware->pkt_number != buffer_t.pkt_number) {
			fclose(file);
			file = NULL;
			return -1;
		}

		result = fwrite(Firmware->buffer, 1, Firmware->length,file);
		DBG("result %d\n", result);
		DBG("firmware_length %d\n", Firmware->length);
		hexdump("buffer", Firmware->buffer, 512, 1);
		if (result != Firmware->length) {
			perror("error writing to file");	   
			fclose(file);
			file = NULL;
			return EXIT_FAILURE;
		}

		Firmware->index = 0;

		fclose(file);
		file = NULL;
		return 1;
	}

        return 0;	
}
int FileToDelete()
{

	const char *fileToDelete = "/tmp/g3update-M.m.n.tgz";
	char command[256];

	snprintf(command, sizeof(command), "rm %s", fileToDelete);

	int result = system(command);

	if(result == -1){
		perror("system");
	}
	const char *script_path = "/usr/sbin/candown.sh";
	
	mode_t mode = S_IRWXU | S_IRWXG | S_IRWXO;

	if (chmod(script_path, mode) == -1) {
		perror("chmod");
		return -1;
	}

	int ret = system(script_path);
	if (ret == -1) {
		perror("system");
 		return -1;
	}
	
	return 0;
}

int Firmware_Update_Script()
{
	const char *script_path = "/usr/sbin/fwupdate.sh";
	
	mode_t mode = S_IRWXU | S_IRWXG | S_IRWXO;

	if (chmod(script_path, mode) == -1) {
		perror("chmod");
		return -1;
	}

	int ret = system(script_path);
	if (ret == -1) {
		perror("system");
 		return -1;
	}

	return 0;	
}

int Create_Bridge()
{
	const char *script_path = "/usr/sbin/usbbridge.sh";
	
	mode_t mode = S_IRWXU | S_IRWXG | S_IRWXO;

	if (chmod(script_path, mode) == -1) {
		perror("chmod");
		return -1;
	}

	int ret = system(script_path);
	if (ret == -1) {
		perror("system");
 		return -1;
	}

	return 0;	
}

int Disable_Bridge()
{
	const char *script_path = "/usr/sbin/Disablebridge.sh";
	
	mode_t mode = S_IRWXU | S_IRWXG | S_IRWXO;

	if (chmod(script_path, mode) == -1) {
		perror("chmod");
		return -1;
	}

	int ret = system(script_path);
      if (ret == -1) {
		perror("system");
		return -1;
	}
	
	return 0 ;
}

ssize_t host_write(const void *buf,size_t count)
{
	if(HFCPComMode == HFCP_COM_MODE_USB) {
		DBG("Garuda in USB HFCPComMode");
		led_set(LED_USB_LED, LED_OFF);
		led_set(LED_USB_LED, LED_ON);
		led_set(LED_WLANBT_LED, LED_OFF);
		return usbhid_write(buf,count);
	}
	else if(HFCPComMode == HFCP_COM_MODE_WIFI) {
		DBG("Garuda in WiFi HFCPComMode.");
		led_set(LED_USB_LED, LED_OFF);
		led_set(LED_WLANBT_LED, LED_OFF);
		led_set(LED_WLANBT_LED, LED_ON);
		return net_if_write(buf,HFCP_PKT_SIZE);
	}
	else {
		/* Error Condition.*/
		DBG("Garuda in ERROR HFCPComMode");
		led_set(LED_USB_LED, LED_OFF);
		led_set(LED_USB_LED, LED_ON);
		led_set(LED_WLANBT_LED, LED_OFF);
		return 0;
	}
}

int handle_hfcp_rx(uint8_t * buf, int length)
{
	hfcpReq_t *hfcp = (hfcpReq_t *) buf;
	hfcpResp_t *pAck = NULL;
        int ret;
	DBG("proto_id %d\n", hfcp->proto_id);
	switch (hfcp->proto_id) {
	case SESSION_MANAGEMENT:
		DBG("SESSION_MANAGEMENT");
		process_session_mgmt_command((uint8_t *) hfcp);
		break;
	case CAN_PROTOCOL_ID:
	case CAN_CH1_PROTO_ID:
	case ISO15765_PROTO_ID:
	case ISO15765_CH1_PROTO_ID:
		DBG("CAN_PROTOCOL_ID");
		process_CAN_command(buf);
		break;
	case CAN_FD_PROTOCOL_ID:
	case ISO15765_FD_PROTO_ID:
	case CAN_FD_CH1_PROTO_ID:
	case ISO15765_FD_CH1_PROTO_ID:
	case CAN_FD_PS:
		DBG("CAN_FD_PROTOCOL_ID");
		process_CAN_FD_command(buf);
		break;
	case KWP_PROTOCOL_ID:
	case ISO_9141_PROTO_ID:
	//	process_KWP_command(&data_read[0]);
		process_KWP_command(buf);
		break;
	case J1708_PROTOCOL_ID:
		break;
	case SETUP_CMD_ID:
		DBG("SETUP_CMD_ID");
		process_Setup_command((uint8_t *) hfcp);
		break;
	case FIRMWARE_UPGRADE_MODE:
		DBG("Firmware_Upgrade");
		ret = Firmware_Update_command(buf, USB_PKT_SIZE);
		pAck = (hfcpResp_t *) (&(hfcp->u));
		if (ret < 0 ){
			FileToDelete();
			pAck->status = ERR_NOT_SUPPORTED;
	        }
                pAck->status = STATUS_NOERROR;
                if (host_write((void *)hfcp, USB_PKT_SIZE) <= 0){
	            DBG("write faile: %s", strerror(errno));
                }		    
		break;
	case FIRMWARE_SCRIPT:
		ret = Firmware_Update_Script();                		
		pAck = (hfcpResp_t *) (&(hfcp->u));
		if (ret < 0 ){
			pAck->status = ERR_NOT_SUPPORTED;
	        }
		pAck->status = STATUS_NOERROR;
                if (host_write((void *)hfcp, USB_PKT_SIZE) <= 0){
	            DBG("write faile: %s", strerror(errno));
                }		    
		break;
	case TMP_FILE_DELETE:
		ret = FileToDelete();
		pAck = (hfcpResp_t *) (&(hfcp->u));
		if (ret < 0 ){
			pAck->status = ERR_NOT_SUPPORTED;
	        }
		pAck->status = STATUS_NOERROR;
                if (host_write((void *)hfcp, USB_PKT_SIZE) <= 0){
	            DBG("write faile: %s", strerror(errno));
                }		    
		break;
	case Enable_RNDIS:
		ret = Create_Bridge();
		pAck = (hfcpResp_t *) (&(hfcp->u));
		if (ret < 0 ){
			pAck->status = ERR_NOT_SUPPORTED;
	        }
		pAck->status = STATUS_NOERROR;
                if (host_write((void *)hfcp, USB_PKT_SIZE) <= 0){
	            DBG("write faile: %s", strerror(errno));
                }		    
		break;
	case Disable_RNDIS:
		ret = Disable_Bridge();
		pAck = (hfcpResp_t *) (&(hfcp->u));
		if (ret < 0 ){
			pAck->status = ERR_NOT_SUPPORTED;
	        }
		pAck->status = STATUS_NOERROR;
                if (host_write((void *)hfcp, USB_PKT_SIZE) <= 0){
	            DBG("write faile: %s", strerror(errno));
                }		    
		break;	        	
	case J1939_PROTOCOL_ID:
	case J1939_CH1_PROTOCOL_ID:
		//process_J1939_command(&data_read[0]);
		break;
	case DOIP_PROTOCOL_ID:
		process_doip_command(hfcp);
		break;
	default:
		DBG("NOT_SUPPORTED");
		pAck = (hfcpResp_t *) (&(hfcp->u));
		pAck->status = ERR_NOT_SUPPORTED;
		if (host_write((void *)hfcp, USB_PKT_SIZE) <= 0) {
			DBG("Write failed: %s", strerror(errno));
		}
	}

	return 0;
}
