
/*
 * @file iso14230.c 
 *
 * Copyright (c) 2023-2024, Capgemini - Intelligent Devices
 * All Rights Reserved.
 *
 * The information contained herein is confidential property of Capgemini.
 * The use, copying, transfer or disclosure of such information is
 * prohibited except by express written agreement with Capgemini.
 */

/******************************************************************************
* P U R P O S E: This module implements ISO 14230 protocol
*******************************************************************************/

/******************************************************************************
*                    I N C L U D E   F I L E S
*******************************************************************************/
#include "iso14230.h"
#include "app_iso14230.h"
#include "j2534_filter.h"
#include "hfcp.h"

#define FALSE     0
#define TRUE      1

//extern void configureUart( UART_InitTypeDef *uartConfig);
const char *devicePath;

//static UART_InitTypeDef UAR_InitStruct;

/******************************************************************************
*               E X P O R T E D   D E F I N I T I O N S
*******************************************************************************/

/* GPIO PINS Used */
typedef enum {
	KW_TXD,			/* GPIO Port A Pin 0 */
	KW_RXD,			/* GPIO Port A Pin 1 */
	KLINE_PULLUP_SLCT,	/* GPIO Port D Pin 3 */
	KLINE_LLINE_SLCT,	/* GPIO Port D Pin 4 */
	LLINE_ISO_SLCT,		/* GPIO Port D Pin 5 */
	KLINE_ISO_SLCT,		/* GPIO Port D Pin 6 */
	KW_TX_5BAUD		/* GPIO Port D Pin 7 */
} KWP_GPIOs_t;

/******************************************************************************
*               I N T E R N A L   D E F I N I T I O N S
*******************************************************************************/

/******************************************************************************
*         P R I V A T E    F U N C T I O N    P R O T O T Y P E S
*******************************************************************************/
static void ISO9141_14230_LinkInit(void);
static bool ISO9141_14230_GetParity(void);
static uint8_t ISO9141_14230_GetChecksum(const uint8_t * p_Data,
					 uint16_t p_Length, uint8_t p_TxRx);
static ISO9141_14230_QSTATUS ISO9141_14230_AddToQ(ISO9141_14230_QTYPE
						  p_QType,
						  const ISO9141_14230_Q_S *
						  p_ISO9141_14230Q_SP,
						  uint16_t p_dataoffset);
static ISO9141_14230_QSTATUS ISO9141_14230_DelFromQ(ISO9141_14230_QTYPE p_QType,
						    ISO9141_14230_Q_S *
						    p_ISO9141_14230Q_SP);
static ISO9141_14230_QSTATUS ISO9141_14230_ReadFromQ(ISO9141_14230_QTYPE
						     p_QType,
						     ISO9141_14230_Q_S *
						     p_ISO9141_14230Q_SP);
static ISO9141_14230_QSTATUS ISO9141_14230_GetByteFromQ(ISO9141_14230_QTYPE
							p_QType,
							uint16_t p_Index,
							uint8_t *
							p_RetDataByte);
static void ISO9141_14230_DisableBusMonitor(void);
static void ISO9141_14230_EnableBusMonitor(void);

/* static void vKwpTimerCallback( TimerHandle_t pxTimer );*/
//__irq __arm void GPIOPinChangeISR (void);

/******************************************************************************
*                   P R I V A T E   V A R I A B L E S
*******************************************************************************/

/* Tx Q */
static volatile uint8_t l_ISO9141_14230_TxQ[ISO9141_14230_MAXTXQSIZE];

/* Rx Q */
static volatile uint8_t l_ISO9141_14230_RxQ[ISO9141_14230_MAXRXQSIZE];

/* RxTimestamp Queue */
static volatile uint8_t l_ISO9141_14230_RxTimeQ[ISO9141_14230_MAXRXTIMEQSIZE];

/* RxLength Queue */
static volatile uint8_t l_ISO9141_14230_RxLenQ[ISO9141_14230_MAXRXLENQSIZE];

/* Init Link Information */
static volatile ISO9141_14230_LinkInit_S l_ISO9141_14230LinkInit_S;

/* Init Link Return information */
static volatile ISO9141_14230_LinkInitRet_S l_ISO9141_14230LinkInitRet_S;

/* ISO 9141 / 14230 Parameters */
static volatile uint32_t l_RefIdleTime;	/* Reference Idle time */
static volatile uint32_t l_RefInitTime;	/* Reference Init time */
static uint32_t l_5BaudIdleTime;	/* 5 Baud idle time - W0 or W5 */
static uint32_t l_loopback;	/* Loopback enabled? 1- Yes, 0 - NO */
static uint32_t l_P1MAX;	/* P1 max timing parameter */
static uint32_t l_P3MIN;	/* P3 min timing parameter */
static uint32_t l_P4MIN;	/* P4 min timing parameter */
static uint32_t l_W0;		/* W0 idle time */
static uint32_t l_W1;		/* W1 timing parameter */
static uint32_t l_W2;		/* W2 timing parameter */
static uint32_t l_W3;		/* W3 timing parameter */
static uint32_t l_W4;		/* W4 timing parameter */
static uint32_t l_W5;		/* W5 idle time */
static uint32_t l_TIDLE;	/* Idle time for Fast init */
static uint32_t l_TINIL;	/* Low time for Fast init
				   pattern */
static uint32_t l_TWUP;		/* Wakeup time for Fast init
				   pattern */
static uint32_t l_BaudRate;	/* Baud rate for UART */
static volatile uint16_t l_UARTIntrTxIndex;	/* UART Interrupt Tx length
						   calculation */
static volatile uint16_t l_RxLength;	/* Receive message length */
static volatile uint16_t l_TxLength;	/* Transmit message length */
static volatile uint16_t l_TxIndex;	/* Tx byte index */
static uint8_t l_ProtocolId;	/* Protocol Id */
static uint8_t l_Parity;	/* Parity configuration */
static uint8_t l_DataBits;	/* Number of data bits */
static uint8_t l_FiveBaudMod;	/* Five baud mode */
static volatile uint8_t l_InitStatus;	/* Status of Initialization of
					   link */
static volatile uint8_t l_BitPos;	/* Bit position for 5 Baud or
					   Fast Init */
static uint8_t l_ConnectFlags;	/* Connect Flags */
static uint16_t l_LengthByte;	/* Length byte index */
static uint16_t l_PktLength;	/* Defines the packet length of Handling data wrt length */
static uint32_t l_P1MAX_ExtendedTimeout;	/* Extended P1Max to find the maximum timeout for P1_Max */

/* Queue variables */

/* Rx Queue */
static volatile uint16_t l_RxQFront;
static volatile uint16_t l_RxQRear;

/* Tx Queue */
static volatile uint16_t l_TxQFront;
static volatile uint16_t l_TxQRear;

/* RxLength Queue */
static volatile uint16_t l_RxLenQFront;
static volatile uint16_t l_RxLenQRear;

/* RxTime Queue */
static volatile uint16_t l_RxTimeQFront;
static volatile uint16_t l_RxTimeQRear;
static volatile bool l_P3MINTimeout;	/* P3Min timeout indication */
static volatile bool l_MsgReceived;	/* Message Received Indication */
static volatile bool l_TxComp;	/* Tx completion indication */
static uint8_t l_InitTimer;	/* ISO 9141 / 14230 Timer for
				   Initialization */

/* static TimerHandle_t kwpTimerHandler       ; */

/* FreeRTOS's returned timer handler, used for uart time guard and receive time out */

static int uartFd;
static int baudrate;
static int dataLength;
const char *devicePath;
static uint32_t ISO_9141_OR_14230 = 0; // kwp_ch
 
/******************************************************************************
*                   E X P O R T E D   F U N C T I O N S
*******************************************************************************/

/******************************************************************************
* Function name     : void ISO9141_14230_Init(ISO9141_14230_Init_S
                                                        *p_ISO9141_14230Init_SP)
*    returns        : ISO9141_14230_RETCODE
*    arg1           : Init structure
* Description       : This funtions performs the initialization of communication
                      hardware
* Notes             : restrictions, odd modes
*******************************************************************************/
void ISO9141_14230_Init(const ISO9141_14230_Init_S * p_ISO9141_14230Init_S)
{
	/* UART structure */
	UART_InitTypeDef UART_InitStruct;

	/* Initialize Transmission Queue */
	memset((void *)&l_ISO9141_14230_TxQ, 0, (sizeof(uint8_t) * ISO9141_14230_MAXTXQSIZE));	//memset() is used to fill a block of memory with a particular value.

	/* Initialize Reception Queue */
	memset((void *)&l_ISO9141_14230_RxQ, 0,
	       (sizeof(uint8_t) * ISO9141_14230_MAXRXQSIZE));

	/* Initialize Rx Length Queue */
	memset((void *)&l_ISO9141_14230_RxLenQ, 0,
	       (sizeof(uint8_t) * ISO9141_14230_MAXRXLENQSIZE));

	/* Initialize Rx Time Queue */
	memset((void *)&l_ISO9141_14230_RxTimeQ, 0,
	       (sizeof(uint8_t) * ISO9141_14230_MAXRXTIMEQSIZE));

	/* Initialize Link Init structure */
	memset((void *)&l_ISO9141_14230LinkInit_S, 0,
	       sizeof(ISO9141_14230_LinkInit_S));

	/* Initialize Init Return data structure */
	memset((void *)&l_ISO9141_14230LinkInitRet_S, 0,
	       sizeof(ISO9141_14230_LinkInitRet_S));

	/* Queue variables */

	/* Rx Queue */
	l_RxQFront = 0;
	l_RxQRear = 0;

	/* Tx Queue */
	l_TxQFront = 0;
	l_TxQRear = 0;

	/* RxLength Queue */
	l_RxLenQFront = 0;
	l_RxLenQRear = 0;

	/* RxTime Queue */
	l_RxTimeQFront = 0;
	l_RxTimeQRear = 0;

	/* Save all the default values to ISO 9141 / 14230 parameters */
	l_ProtocolId = p_ISO9141_14230Init_S->ProtocolId;
	l_BaudRate = p_ISO9141_14230Init_S->Baudrate;
	l_ConnectFlags = 0;
	l_Parity = NO_PARITY;
	l_DataBits = EIGHT_BITS;
	l_P1MAX = 20000;
	l_P3MIN = 55000;	/* P3 min is assumed to be used as T4 in case
				   of ISO 9141. T4 specifies the minimum
				   time for diag data request after
				   initialization */
	l_P4MIN = 5000;
	l_W0 = 300000;
	l_W1 = 300000;
	l_W2 = 20000;
	l_W3 = 20000;
	l_W4 = 49500;		/* Minimum Time of 50ms is not possible
				   because of the delay in trasnmission
				   and the max time being 50ms as well */
	l_W5 = 300000;
	l_TIDLE = 300000;
	l_TINIL = 25000;
	l_TWUP = 50000;
	l_FiveBaudMod = FIVEBAUD_MODE_ZERO;
	l_BitPos = 0;
	l_RxLength = 0;
	l_TxLength = 0;
	l_TxIndex = 0;
	l_RefInitTime = 0;
	l_UARTIntrTxIndex = 0;
	l_P3MINTimeout = !FALSE;
	l_MsgReceived = FALSE;
	l_TxComp = FALSE;

	/* Bit code the Connect flags specific to ISO 9141 / 14230 */

	/* K Line only flag */
	if (CHECK_BITU32
	    (p_ISO9141_14230Init_S->Flags,
	     ISO9141_K_LINE_ONLY_BIT_POS) != FALSE) {

		/* Bit 0 */
		SET_BITU8(l_ConnectFlags, BIT_LINECONF);
	}

	/* Checksum flag:to ensure that the correct data is received and that the ECU is not damaged by corrupted messages. */
	if (CHECK_BITU32
	    (p_ISO9141_14230Init_S->Flags,
	     ISO9141_NO_CHECKSUM_BIT_POS) != FALSE) {

		/* Bit 1 */
		SET_BITU8(l_ConnectFlags, BIT_CHKSUM);
	}
#if 0				/* LPC2468 */
	/* Initialize UART - Default configuration */
	UART_InitStruct.US_CHRL = 8;
	UART_InitStruct.US_NBSTOP = 1;
	UART_InitStruct.US_PAR = UART_PARITY_DISABLE;
	UART_InitStruct.US_FIFO_USE = DISABLE;

	/* Load the required baudrate */
	UART_InitStruct.US_BAUDRATE = l_BaudRate;

#endif				/* LPC2468 */

/*
	UART_InitStruct.baudrate = l_BaudRate;
	UART_InitStruct.dataLen = 8;
	UART_InitStruct.stopbits = UART_STOPBIT_1;
	UART_InitStruct.parity = UART_PARITY_DISABLE;
	UART_InitStruct.use_fifo = 0;
	UART_InitStruct.flowcontrol = UART_FLOWCONTROL_NONE;
	UART_InitStruct.rxQueueSz = 2700;
	UART_InitStruct.txQueueSz = 2700;
*/
	/* Init UART */
	UART_MidInit(UART_CH_ID_1, UART_InitStruct);

	/*arg1: specifying UART channel to be initialized
	   arg2: struct containing initialization settings for UART */
	/* Initialize Timeout to 0 */
	UART_Change_RxTimeOut(UART_CH_ID_1, 0);

	/*purpose: to change the receive timeout for UART channnel
	   arg1: UART channel for which the receive timeout is being modified
	   arg2: receive timeout is being set to ) */
	/* Initialize Time Guard to 0 */
	UART_Change_TimeGuard(UART_CH_ID_1, 0);

	/* Interrupt handling */

	/* Configure GPIOs as O/P */

/*
	Config_Pin_Output(KLINE_LLINE_SLCT);
	Config_Pin_Output(KLINE_PULLUP_SLCT);
	Config_Pin_Output(LLINE_ISO_SLCT);
	Config_Pin_Output(KLINE_ISO_SLCT);
	Config_Pin_Output(KW_TX_5BAUD);
*/

	/*
	 * Clear Buffers: Clearing the RX queue typically involves discarding
	 * any pending or buffered received data in the queue, effectively
	 * emptying it.
	 * This operation can be useful in scenarios where you want to start
	 * fresh or ensure that only the most recent data is processed.
	 */
	//uartClearRxQueue(UART_CH_ID_1,NORMAL_MODE);
	uartClearRxQueue(devicePath, NORMAL_MODE);

	//uartClearTxQueue(const char *devicePath,NORMAL_MODE)
	uartClearTxQueue(devicePath, NORMAL_MODE);

	/* Configure the PinChange Interrupt */
	// AMIT IMP KWP_RX_GPIO_extIntConfig();

	/* Update the Init Link status */
	l_InitStatus = NO_LINKINIT;

	/* Set KW_TX_5BAUD Signal to Enable Transmission */
	//  Set_Pin_High(KW_TX_5BAUD);

	/* Initially disable L line. Use K line only till Link Init is done */
	//  Set_Pin_Low(KLINE_LLINE_SLCT);

	/* Based on Battery voltage select the appropriate pullup ????? */

	// if (CheckIf_VBATTis24V() == TRUE) {
	//      // Set the pullup for K line ??????
	//      Set_Pin_Low(KLINE_PULLUP_SLCT);
	// } else {
	/* Set the pullup for K line ?????? */
	//  Set_Pin_High(KLINE_PULLUP_SLCT);
	// }    
	/* Connect K & L to OBD Connector */
	// Set_Pin_High(LLINE_ISO_SLCT);
	// Set_Pin_High(KLINE_ISO_SLCT);

	/*
	 * Start Timer for Timestamp for 14230 and obtain the corresponding
	 * timer id
	 */
	// timestamp_id[GARUDA_KWP_CH1] = start_time_stamp();
	// kwpTimerHandler = xTimerCreate("kwpTimer", portTICK_PERIOD_MS, pdTRUE, (void *)KWP_TIMER_ID, vKwpTimerCallback);
	// Timer_Init(); 

}

/******************************************************************************
* Function name     : ISO9141_14230_RETCODE ISO9141_14230_Command(
                                                   ISO9141_14230_Cmd_S
                                                   *p_ISO9141_14230Cmd_SP)
*    returns        : ISO9141_14230_RETCODE
*    arg1           : ISO9141_14230_Cmd_S *p_ISO9141_14230Cmd_SP:
                      Command Structure pointer
* Description       : This funtions performs various command executions to
                      configure ISO 9141 / 14230 Communication
* Notes             : restrictions, odd modes
*******************************************************************************/
ISO9141_14230_RETCODE ISO9141_14230_Command(ISO9141_14230_Cmd_S
					    * p_ISO9141_14230Cmd_SP)
{
	ISO9141_14230_RETCODE fl_RetCode = NO_ERROR;
	uint8_t fl_ChkSum = CHECKSUM_OK;

	/* Switch to the appropriate Command based on the IOCtl Id */
	/* All P and W time parameters are in us with resolution
	   applied */
	switch (p_ISO9141_14230Cmd_SP->IOCtlId) {

	case GET_CONFIG:	/* Get Config */

		/* Obtain the value of the appropriate configuration parameter */
		switch (p_ISO9141_14230Cmd_SP->ParamId) {
		case DATA_RATE:
				/* Obtain the Baud rate and load it onto the command data
				   location */
				*(p_ISO9141_14230Cmd_SP->pData) = l_BaudRate;
				break;
		case LOOPBACK:
				*(p_ISO9141_14230Cmd_SP->pData) = l_loopback;
				break;
		case P1_MAX:
				/* Obtain P1 max and load it onto the command data
				   location */
				*(p_ISO9141_14230Cmd_SP->pData) =
				    (uint32_t) l_P1MAX / 500;
				break;
		case P3_MIN:
				/* Obtain P3 min and load it onto the command data
				   location */
				*p_ISO9141_14230Cmd_SP->pData = l_P3MIN / 500;
				break;
		case P4_MIN:
				/* Obtain P4 min and load it onto the command data
				   location */
				*p_ISO9141_14230Cmd_SP->pData = l_P4MIN / 500;
				break;
		case W0:
				/* Obtain W0 and load it onto the command data
				   location */
				*p_ISO9141_14230Cmd_SP->pData = l_W0 / 1000;
				break;
		case W1:
				/* Obtain W1 and load it onto the command data
				   location */
				*p_ISO9141_14230Cmd_SP->pData = l_W1 / 1000;
				break;
		case W2:
				/* Obtain W2 and load it onto the command data
				   location */
				*p_ISO9141_14230Cmd_SP->pData = l_W2 / 1000;
				break;
		case W3:
				/* Obtain W3 and load it onto the command data
				   location */
				*p_ISO9141_14230Cmd_SP->pData = l_W3 / 1000;
				break;
		case W4:
				/* Obtain W4 and load it onto the command data
				   location */
				*p_ISO9141_14230Cmd_SP->pData = l_W4 / 1000;
				break;
		case W5:
				/* Obtain W5 and load it onto the command data
				   location */
				*p_ISO9141_14230Cmd_SP->pData = l_W5 / 1000;
				break;
		case TIDLE:
				/* Obtain TIDLE and load it onto the command data
				   location */
				*p_ISO9141_14230Cmd_SP->pData = l_TIDLE / 1000;
				break;
		case TINIL:
				/* Obtain TINIL and load it onto the command data
				   location */
				*p_ISO9141_14230Cmd_SP->pData = l_TINIL / 1000;
				break;
		case TWUP:
				/* Obtain TWUP and load it onto the command data
				   location */
				*p_ISO9141_14230Cmd_SP->pData = l_TWUP / 1000;
				break;
		case PARITY:
				/* Obtain the Parity and load it onto the command data
				   location */
				*p_ISO9141_14230Cmd_SP->pData = l_Parity;
				break;
		case DATA_BITS:
				/* Obtain the Data Bits and load it onto the command data
				   location */
				if (l_DataBits == EIGHT_BITS) {
					*p_ISO9141_14230Cmd_SP->pData = 0;
				}

				else {
					*p_ISO9141_14230Cmd_SP->pData = 1;
				}
				break;
		case FIVE_BAUD_MOD:
				/* Obtain the Five Baud Mode and load it onto the command
				   data location */
				*p_ISO9141_14230Cmd_SP->pData = l_FiveBaudMod;
				break;
		default:
				/* Invalid Param Id */
				fl_RetCode = INVALID_PARAMETERID;
				break;
		}
		break;
	case SET_CONFIG:	/* Set Config */
		/* Set the value of the appropriate configuration parameter */
		switch (p_ISO9141_14230Cmd_SP->ParamId) {
		case DATA_RATE:

				/* Set the Baud rate */
				l_BaudRate = *p_ISO9141_14230Cmd_SP->pData;

				/*if(125000 == l_BaudRate)
				   {
				   l_BaudRate = 124800;
				   }
				   else
				   {

				   } */

				/* Clear temporary Tx and Rx variable */
				uartClearTxQueue(devicePath, NORMAL_MODE);
				uartClearRxQueue(devicePath, NORMAL_MODE);

				//enable_uart(UART_CH_ID_1);
//                 UART_MidEnable(UART_CH_ID_1);
				uartMidEnable(UART_CH_ID_1);

				/* Clear intermediate frame bytes if any */
				l_UARTIntrTxIndex = 0;

				/* Clear Intermediate Rx byte length ????? */
				l_RxLength = 0;

				/* Clear Tx length ????? */
				l_TxLength = 0;
				l_MsgReceived = FALSE;
				l_P3MINTimeout = !FALSE;
				l_TxComp = FALSE;

				/* Configure UART baud rate */

				//UART_Set_Baudrate(UART_CH_ID_1, l_BaudRate);
				UART_Set_Baudrate(uartFd, baudrate);
				break;
		case LOOPBACK:
				l_loopback = *(p_ISO9141_14230Cmd_SP->pData);
				break;
		case P1_MAX:
				/* Set P1 max */
				l_P1MAX = *p_ISO9141_14230Cmd_SP->pData * 500;
				break;
		case P3_MIN:
				/* Set P3 min */
				l_P3MIN = *p_ISO9141_14230Cmd_SP->pData * 500;
				break;
		case P4_MIN:
				/* Set P4 min */
				l_P4MIN = 0;	/* To Increase Flashing Speed */

				/*l_P4MIN = *p_ISO9141_14230Cmd_SP->pData * 500; */
				break;
		case W0:
				/* Set W0 */
				l_W0 = *p_ISO9141_14230Cmd_SP->pData * 1000;
				break;
		case W1:
				/* Set W1 */
				l_W1 = *p_ISO9141_14230Cmd_SP->pData * 1000;
				break;
		case W2:
				/* Set W2 */
				l_W2 = *p_ISO9141_14230Cmd_SP->pData * 1000;
				break;
		case W3:
				/* Set W3 */
				l_W3 = *p_ISO9141_14230Cmd_SP->pData * 1000;
				break;
		case W4:
				/* Set W4 */
				l_W4 = *p_ISO9141_14230Cmd_SP->pData * 1000;
				break;
		case W5:
				/* Set W5 */
				l_W5 = *p_ISO9141_14230Cmd_SP->pData * 1000;
				break;
		case TIDLE:
				/* Set TIDLE */
				l_TIDLE = *p_ISO9141_14230Cmd_SP->pData * 1000;
				break;
		case TINIL:
				/* Set TINIL */
				l_TINIL = *p_ISO9141_14230Cmd_SP->pData * 1000;
				break;
		case TWUP:
				/* Set TWUP */
				l_TWUP = *p_ISO9141_14230Cmd_SP->pData * 1000;
				break;
		case PARITY:
				/* Set Parity */
				l_Parity = *p_ISO9141_14230Cmd_SP->pData;

				/* Clear temporary Tx and Rx variable */
				uartClearRxQueue(devicePath, NORMAL_MODE);
				uartClearTxQueue(devicePath, NORMAL_MODE);

				//enable_uart(UART_CH_ID_1);
				// UART_MidEnable(UART_CH_ID_1);
				uartMidEnable(UART_CH_ID_1);

				/* Clear intermediate frame bytes if any */
				l_UARTIntrTxIndex = 0;

				/* Clear Intermediate Rx byte length ???? */
				l_RxLength = 0;

				/* Clear Tx length ???? */
				l_TxLength = 0;
				l_MsgReceived = FALSE;

				//l_P3MINTimeout = !FALSE;
				l_TxComp = FALSE;

				/* Configure UART Parity */
				if (l_Parity == ODD_PARITY) {
					UART_Set_Parity(UART_CH_ID_1,
							UART_PARITY_ODD);
				}

				else if (l_Parity == EVEN_PARITY) {
					UART_Set_Parity(UART_CH_ID_1,
							UART_PARITY_EVEN);
				}

				else {
					UART_Set_Parity(UART_CH_ID_1,
							UART_PARITY_DISABLE);
				}
				break;
		case DATA_BITS:
				/* Clear temporary Tx and Rx variable */
				uartClearRxQueue(devicePath, NORMAL_MODE);
				uartClearTxQueue(devicePath, NORMAL_MODE);

				//enable_uart(UART_CH_ID_1);
				// UART_MidEnable(UART_CH_ID_1);
				uartMidEnable(UART_CH_ID_1);

				/* Clear intermediate frame bytes if any */
				l_UARTIntrTxIndex = 0;

				/* Clear Intermediate Rx byte length ??? */
				l_RxLength = 0;

				/* Clear Tx length ??? */
				l_TxLength = 0;
				l_MsgReceived = FALSE;
				l_P3MINTimeout = !FALSE;
				l_TxComp = FALSE;

				/* Set Data Bits */
				if (*p_ISO9141_14230Cmd_SP->pData == 0) {
					l_DataBits = EIGHT_BITS;

					/* Configure UART Databits */
					// UART_Set_DataLength(UART_CH_ID_1,8);
					UART_Set_DataLength(uartFd, dataLength);
				}

				else {
					l_DataBits = SEVEN_BITS;

					/* Configure UART Databits */
					// UART_Set_DataLength(UART_CH_ID_1,7);
					UART_Set_DataLength(uartFd, dataLength);
				}
				break;
		case FIVE_BAUD_MOD:
				/* Set Five Baud mode */
				l_FiveBaudMod = *p_ISO9141_14230Cmd_SP->pData;
				break;
		default:

				/* Invalid Param Id */
				fl_RetCode = INVALID_PARAMETERID;
		}
		break;

	case FIVE_BAUD_INIT:
	case FAST_INIT:	/* Link Initialization - FIVE_BAUD_INIT or FAST_INIT */

		/* Load the init information from the Cmd structure to the Link
		   init structure */

		/* Update the Init type */
		l_ISO9141_14230LinkInit_S.InitType =
		    p_ISO9141_14230Cmd_SP->IOCtlId;

		/* Update init data Length */
		l_ISO9141_14230LinkInit_S.Length =
		    p_ISO9141_14230Cmd_SP->Length;

		/* Update the Init Data  */
		memcpy((void *)&l_ISO9141_14230LinkInit_S.Data,
		       (uint8_t *) p_ISO9141_14230Cmd_SP->pData,
		       sizeof(uint8_t) * l_ISO9141_14230LinkInit_S.Length);

		/* Update checksum if configured */
		if ((CHECK_BITU8(l_ConnectFlags, BIT_CHKSUM) == FALSE)
		    && (l_ISO9141_14230LinkInit_S.InitType == FAST_INIT)) {

			/* Obtain the checksum for Init data */
			fl_ChkSum = ISO9141_14230_GetChecksum((void *)
							      l_ISO9141_14230LinkInit_S.Data, l_ISO9141_14230LinkInit_S.Length, CHKSUM_TXDATA);

			/* Store the checksum as the last byte */
			l_ISO9141_14230LinkInit_S.Data
			    [l_ISO9141_14230LinkInit_S.Length] = fl_ChkSum;

			/* Update the length for the checksum */
			l_ISO9141_14230LinkInit_S.Length++;
		}

		/* Call the init function to perform the initialization of the
		   communication link */
		ISO9141_14230_LinkInit();	//To initialize communication link parameters for 5 Baud / Fast Init in Linux, we can use the stty command in the terminal.
		break;

	case CLEAR_TX_BUFFER:	/* Clear Tx Buffers */

		/* Set Length to 0 */
		p_ISO9141_14230Cmd_SP->Length = 0;

		/* Clear all Tx Queues and Structures */
		memset((void *)&l_ISO9141_14230_TxQ, 0,
		       (sizeof(uint8_t) * ISO9141_14230_MAXTXQSIZE));

		/* Clear temporary Tx and Rx variable */
		uartClearRxQueue(devicePath, NORMAL_MODE);
		uartClearTxQueue(devicePath, NORMAL_MODE);

		//enable_uart(UART_CH_ID_1);
//            UART_MidEnable(UART_CH_ID_1);
		uartMidEnable(UART_CH_ID_1);

		/* Clear intermediate frame bytes if any */
		l_UARTIntrTxIndex = 0;
		l_MsgReceived = FALSE;

		//l_P3MINTimeout = !FALSE;
		l_TxComp = FALSE;

		/* Clear Intermediate Rx byte length */
		l_RxLength = 0;

		/* Clear Tx length */
		l_TxLength = 0;

		/* Clear Tx Index */
		l_TxIndex = 0;

		/* Reset the Front and Rear variables */
		l_TxQFront = 0;
		l_TxQRear = 0;
		break;
	case CLEAR_RX_BUFFER:	/* Clear Rx Buffers */
		/* Set Length to 0 */
		p_ISO9141_14230Cmd_SP->Length = 0;

		/* Clear Reception Queue */
		memset((void *)&l_ISO9141_14230_RxQ, 0,
		       (sizeof(uint8_t) * ISO9141_14230_MAXRXQSIZE));

		/* Clear RxLength Queue */
		memset((void *)&l_ISO9141_14230_RxLenQ, 0,
		       (sizeof(uint8_t) * ISO9141_14230_MAXRXLENQSIZE));

		/* Clear RxTime Queue */
		memset((void *)&l_ISO9141_14230_RxTimeQ, 0,
		       (sizeof(uint8_t) * ISO9141_14230_MAXRXTIMEQSIZE));

		/* Clear temporary Tx and Rx variable */
		uartClearRxQueue(devicePath, NORMAL_MODE);
		uartClearTxQueue(devicePath, NORMAL_MODE);

		//enable_uart(UART_CH_ID_1);
//            UART_MidEnable(UART_CH_ID_1);
		uartMidEnable(UART_CH_ID_1);

		/* Clear intermediate frame bytes if any */
		l_UARTIntrTxIndex = 0;
		l_MsgReceived = FALSE;

		//l_P3MINTimeout = !FALSE;

		/* Clear Intermediate Rx byte length */
		l_RxLength = 0;

		/* Clear Tx length */
		l_TxLength = 0;

		/* Reset the Front and Rear variables */
		l_RxQFront = 0;
		l_RxQRear = 0;
		l_RxLenQFront = 0;
		l_RxLenQRear = 0;
		l_RxTimeQFront = 0;
		l_RxTimeQRear = 0;
		break;
	default:
		/* Invalid command id */
		fl_RetCode = INVALID_COMMAND;
	}

	return fl_RetCode;
}

/******************************************************************************
* Function name     : void ISO9141_14230_TimerIntrHandler(void)
*    returns        : void
*    arg1           : None
* Description       : This funtion is called from within the Timer ISR to
                      perform initialization operations
* Notes             : restrictions, odd modes
*******************************************************************************/
void ISO9141_14230_TimerIntrHandler(void)
{
	uint32_t fl_CurrTime;
	uint16_t fl_Cnt;
	uint8_t fl_ChkSum = CHECKSUM_OK;

	/* Determine the status of Init Link */
	if (l_InitStatus == LINKINIT_PENDING) {

		/* Obtain Current Time */
		//get_time_stamp(l_InitTimer, &fl_CurrTime);
		//get_data_logging_time_stamp(&fl_CurrTime);
		//get_time_stamp(&fl_CurrTime);

		/* Determine the type of initialization */
		if (l_ISO9141_14230LinkInit_S.InitType == FIVE_BAUD_INIT) {

			/* 5 Baud Initialization */

			/* Check if the Bus was idle for W5 or W0. If idle perform 5 Baud
			   initialization else wait till W5 or W0 */
			if ((fl_CurrTime - l_RefIdleTime) >= l_5BaudIdleTime) {

				/* Start bit */
				if (l_BitPos == START_BIT_POS) {

					/* Disable bus monitoring for Idle time. Idle time is
					   achieved so no more updation of Ref Idle time */
					ISO9141_14230_DisableBusMonitor();

					/* Send the Start Bit */
//                    Set_Pin_Low(KW_TXD);

					/* Update the Init reference time */
					l_RefInitTime = fl_CurrTime;

					/* Increment Bit position to Transmit the next bit */
					l_BitPos++;
				}

				/* Send Address byte */

				/* Check if the Bit position neither in Start or Stop bit
				   position and also check if FIVE Baud time has elapsed since
				   the transmission of Start bit */
				else if ((l_BitPos > START_BIT_POS) &&
					 (l_BitPos < STOP_BIT_POS) &&
					 ((fl_CurrTime - l_RefInitTime) >=
					  FIVE_BAUD_TIME)) {

					/* Transmit address bits till the Bit position reaches MSB */
					if (l_BitPos <= l_DataBits) {

						/* Address decoding - Check if the bit is 0 or 1 */
						if (CHECK_BITU8
						    (l_ISO9141_14230LinkInit_S.
						     Data[0],
						     (l_BitPos - 1)) != FALSE) {

							/* Bit is 1 - Set K Line */
							//                          Set_Pin_High(KW_TXD);
						}

						else {

							/* Bit is 0 - Reset K Line */
//                            Set_Pin_Low(KW_TXD);
						}

						/* Increment the bit position to transmit the next bit */
						l_BitPos++;

						/* Update Reference Init time to current time */
						l_RefInitTime = fl_CurrTime;
					}

					/* If MSB is transmitted check for Parity bits */

					/* Check for No Parity */
					else if (l_Parity == NO_PARITY) {

						/* When there is no parity increment the bit position to
						   point to the Stop bit based on the Data Bits */
						if (l_DataBits == EIGHT_BITS) {

							/* Increment the bit position to transmit the stop
							   bit */
							l_BitPos = l_BitPos + 1;
						}

						else {

							/* Increment the bit position to transmit the stop
							   bit */
							l_BitPos = l_BitPos + 2;
						}

						/* RefInit time is not updated inorder to transmit
						   stop bit immediately as there is no Parity */
					}

					/* If Parity is configured then Transmit the Parity bit */
					else {

						/* Based on the Parity configuration determine if the
						   Parity bit is 0 or 1 */
						if (ISO9141_14230_GetParity
						    () != FALSE) {

							/* Parity Bit is 1 - Set K Line */
							//                          Set_Pin_High(KW_TXD);
						}

						else {

							/* Parity Bit is 0 - Reset K Line */
//                            Set_Pin_Low(KW_TXD);
						}

						/* After transmission of Parity bit, increment the bit
						   position to point to the Stop bit based on the
						   Data Bits */
						if (l_DataBits == EIGHT_BITS) {

							/* Increment the bit position to transmit the stop
							   bit */
							l_BitPos = l_BitPos + 1;
						}

						else {

							/* Increment the bit position to transmit the stop
							   bit */
							l_BitPos = l_BitPos + 2;
						}

						/* Update Reference Init time to current time */
						l_RefInitTime = fl_CurrTime;
					}
				}

				else {

					/* Do nothing */
				}

				/* Stop bit */
				/* Transmit Stop bit if the Bit position is in the
				   Stop bit position and also check if the Five baud time has
				   elapsed since the transmission of Last bit of
				   address / parity */
				if ((l_BitPos == STOP_BIT_POS) &&
				    ((fl_CurrTime - l_RefInitTime) >=
				     FIVE_BAUD_TIME)) {

					/* Stop Bit */
//                  Set_Pin_High(KW_TXD);

					/* Update the Init reference time */
					l_RefInitTime = fl_CurrTime;

					/* Increment bit position to indicate no more bits - All
					   bits are sent */
					l_BitPos++;
				}

				/* Perform operations after transmission of Stop bit */
				else if ((l_BitPos == (STOP_BIT_POS + 1))
					 && ((fl_CurrTime - l_RefInitTime)
					     >= FIVE_BAUD_TIME)) {

					/* Check if L line was configured. If yes then disable */
					if (CHECK_BITU8
					    (l_ConnectFlags,
					     BIT_LINECONF) == FALSE) {

						/* Disable L line after Initialization ?????? */
//                    Set_Pin_Low(KLINE_LLINE_SLCT);
					}

					/* Configure the UART Tx as Peripheral */
					//                Config_Pin_UART_Mode(KW_TXD);

					/* Enable UART communication */
					uartClearTxQueue(UART_CH_ID_1,
							 ISR_MODE);
					uartClearRxQueue(UART_CH_ID_1,
							 ISR_MODE);

					//enable_uart(UART_CH_ID_1);
//                    UART_MidEnable(UART_CH_ID_1);
					uartMidEnable(UART_CH_ID_1);

//                    UART_ITConfig (UART_CH_ID_1,IER_THRE,ENABLE);
//                    UART_ITConfig (UART_CH_ID_1,IER_RBR,ENABLE);

					/* Load the UART Receive Timeout W1 - RETTO */
					UART_Change_RxTimeOut(UART_CH_ID_1,
							      l_W1);
					UART_Restart_RxTimeOut(UART_CH_ID_1);

					/* Increment bit position */
					l_BitPos++;
				}

				/* Check for Rx bytes from ECU after completion of Stop bit */
				else if (l_BitPos > (STOP_BIT_POS + 1)) {

					/* Check for data based on Length in the Queue */
					if (ISO9141_14230_DelFromQ
					    (ISO9141_14230_LEN_Q,
					     (ISO9141_14230_Q_S *) &
					     l_ISO9141_14230LinkInitRet_S.
					     Length)
					    != ISO9141_14230_Q_EMPTY) {

						/* Check if Length has reached Address byte position
						   - Key bytes received */
						if (l_ISO9141_14230LinkInitRet_S.Length == ADDRESS_BYTE_INV_POS) {

							/* Obtain Rx data upto the frame Length from the
							   UART */
							for (fl_Cnt = 0;
							     fl_Cnt <
							     l_ISO9141_14230LinkInitRet_S.
							     Length; fl_Cnt++) {

								// UART_MidRecv(UART_CH_ID_1,
								//  (void*)&l_ISO9141_14230LinkInitRet_S.Data[fl_Cnt],1);
								uartMidRecv
								    (UART_CH_ID_1,
								     (uint8_t *)
								     &
								     l_ISO9141_14230LinkInitRet_S.
								     Data
								     [fl_Cnt],
								     1);
							}

							/* Based on the 5 Baud configuration perform
							   operations upon reception of Key bytes */
							if ((l_FiveBaudMod
							     ==
							     FIVEBAUD_MODE_ZERO)
							    ||
							    (l_FiveBaudMod
							     ==
							     FIVEBAUD_MODE_ONE))
							{

								/* In this Mode, receive ECU Pattern and
								   Key bytes, Transmit KeyByte inverse
								   and then Receive Address Inverse byte */

								/* In this Mode, receive ECU Pattern and
								   Key bytes, Transmit KeyByte inverse
								   No reception of Address Inverse byte */

								/* Set the Tx length to indicate Tx byte */
								l_TxLength = 1;

								/* Load Rx timeout W4 before Tx of KB2 inverse
								   - RETTO */
								/* initially it was l_w4 Since it is not time out Changed the timing
								   limit shud be 25 to 50 ms - if changed to only l_W4 5Baud will not work */
								UART_Change_RxTimeOut
								    (UART_CH_ID_1,
								     l_W4 -
								     19500);
								UART_Restart_RxTimeOut
								    (UART_CH_ID_1);
							}

							else if (l_FiveBaudMod
								 ==
								 FIVEBAUD_MODE_TWO)
							{

								/* In this Mode, receive ECU Pattern and
								   Key bytes and receive the Address inverse
								   - No transmission of Key byte 2 inverse */

								/* Clear the Tx length to indicate no Tx byte */
								l_TxLength = 0;

								/* Load Max Rx timeout for Address inverse byte
								   W4 - RETTO */
								UART_Change_RxTimeOut
								    (UART_CH_ID_1,
								     W4_MAX_TIME);
								UART_Restart_RxTimeOut
								    (UART_CH_ID_1);
							}

							else if (l_FiveBaudMod
								 ==
								 FIVEBAUD_MODE_THREE)
							{

								/* In this Mode, receive ECU Pattern and
								   Key bytes - No Tx of Key byte 2 inverse and
								   no Rx of address inverse byte */

								/* Clear the Tx length to indicate no Tx byte */
								l_TxLength = 0;

								/* Clear Rx Length */
								l_RxLength = 0;

								/* Store Protocol Id */
								l_ISO9141_14230LinkInitRet_S.
								    ProtocolId =
								    l_ProtocolId;

								/* Store Init type */
								l_ISO9141_14230LinkInitRet_S.
								    IOCtlId =
								    FIVE_BAUD_INIT;

								/* Store Only Key Bytes - Store the Keybytes at
								   0th and 1st element of the array and modify
								   length to indicate only Keybytes */
								l_ISO9141_14230LinkInitRet_S.
								    Data[0] =
								    l_ISO9141_14230LinkInitRet_S.
								    Data[1];
								l_ISO9141_14230LinkInitRet_S.
								    Data[1] =
								    l_ISO9141_14230LinkInitRet_S.
								    Data[2];
								l_ISO9141_14230LinkInitRet_S.
								    Length = 2;

								/* Report to the Call Back function */
								App_InitData
								    ((void *)
								     &l_ISO9141_14230LinkInitRet_S);

								/* Set the Link Init Status to Done */
								l_InitStatus
								    =
								    LINKINIT_DONE;

								/* Stop Timer for Initialization for
								   ISO 9141/14230 */
								//                          stop_time_stamp(l_InitTimer);

								/* Indicate response bytes received */
								l_MsgReceived
								    = !FALSE;

								/* Load P3 min timeout for minimum time before
								   Data transmission */
								UART_Change_RxTimeOut
								    (UART_CH_ID_1,
								     l_P3MIN);
								UART_Restart_RxTimeOut
								    (UART_CH_ID_1);
							}

							else {

								/* Should not enter else */
							}
						}

						/* Check if length has exceeded Address byte position
						   - Address byte inverse received */
						else if
						    (l_ISO9141_14230LinkInitRet_S.
						     Length >
						     ADDRESS_BYTE_INV_POS) {

							/* Store the Address byte inverse after Key bytes */
							// UART_MidRecv(UART_CH_ID_1,
							//       (void*)&l_ISO9141_14230LinkInitRet_S.Data
							//     [l_ISO9141_14230LinkInitRet_S.Length-1], 1);
							uartMidRecv
							    (UART_CH_ID_1,
							     (uint8_t *) &
							     l_ISO9141_14230LinkInitRet_S.
							     Data
							     [l_ISO9141_14230LinkInitRet_S.
							      Length - 1], 1);

							/* Based on the 5 Baud configuration perform
							   operations upon reception of Address inverse
							   byte */
							if (l_FiveBaudMod ==
							    FIVEBAUD_MODE_ZERO
							    ||
							    l_FiveBaudMod
							    ==
							    FIVEBAUD_MODE_TWO) {

								/* In this FIVEBAUD_MODE_ZERO, receive ECU
								   Pattern and Key bytes, Transmit KeyByte
								   inverse and then Receive Address Inverse
								   byte */

								/* In this FIVEBAUD_MODE_TWO, receive ECU
								   Pattern and Key bytes and receive the
								   Address inverse - No transmission of
								   Key byte 2 inverse */

								/* Store Protocol Id */
								l_ISO9141_14230LinkInitRet_S.
								    ProtocolId =
								    l_ProtocolId;

								/* Store Init type */
								l_ISO9141_14230LinkInitRet_S.
								    IOCtlId =
								    FIVE_BAUD_INIT;

								/* Store Only Key Bytes - Store the Keybytes at
								   0th and 1st element of the array and modify
								   length to indicate only Keybytes */
								l_ISO9141_14230LinkInitRet_S.
								    Data[0] =
								    l_ISO9141_14230LinkInitRet_S.
								    Data[1];
								l_ISO9141_14230LinkInitRet_S.
								    Data[1] =
								    l_ISO9141_14230LinkInitRet_S.
								    Data[2];
								l_ISO9141_14230LinkInitRet_S.
								    Length = 2;

								/* Report to the Call Back function */
								App_InitData
								    ((void *)
								     &l_ISO9141_14230LinkInitRet_S);

								/* Stop Timer for Timestamp for
								   ISO 9141/14230 */
//                                stop_time_stamp(l_InitTimer);

								/* Set the Link Init Status to Done */
								l_InitStatus
								    =
								    LINKINIT_DONE;

								/* Indicate response bytes received */
								l_MsgReceived
								    = !FALSE;

								/* Load P3 min timeout for minimum time before
								   Data transmission */
								UART_Change_RxTimeOut
								    (UART_CH_ID_1,
								     l_P3MIN);
								UART_Restart_RxTimeOut
								    (UART_CH_ID_1);
							}
						}
					}
				}

				else {

					/* Do nothing */
				}
			}
		}

		else {

			/* Fast Init */

			/* Check if the Bus was idle for TIdle. If idle perform Fast
			   initialization else wait till TIdle */
			if ((fl_CurrTime - l_RefIdleTime) >= l_TIDLE) {

				/* Generate Wakeup Pattern */
				if (l_BitPos == WKUP_TINIL) {

					/* Disable bus monitoring for Idle time. Idle time is
					   achieved so no more updation of Ref Idle time */
					ISO9141_14230_DisableBusMonitor();

					/* TiniL pattern */
					//                  Set_Pin_Low(KW_TXD);
					/* Update the Init reference time */
					l_RefInitTime = fl_CurrTime;

					/* Increment l_BitPos to perform rest of the pattern */
					l_BitPos++;
				}

				else if ((l_BitPos == WKUP_TWUP) &&
					 ((fl_CurrTime - l_RefInitTime) >=
					  l_TINIL)) {

					/* TWup - TiniL pattern */
					//                Set_Pin_High(KW_TXD);

					/* Time not updated to represent the rest of Twup */
					/* Increment l_BitPos to indicate end of wakeup pattern */
					l_BitPos++;
				}

				else if ((l_BitPos == WKUP_STARTREQ) &&
					 ((fl_CurrTime - l_RefInitTime) >=
					  l_TWUP)) {

					/* Check if L line was configured. If yes then disable */
					if (CHECK_BITU8
					    (l_ConnectFlags,
					     BIT_LINECONF) == FALSE) {

						/* Disable L line after Initialization ?????? */
//                        Set_Pin_Low(KLINE_LLINE_SLCT);
					}

					/* Configure UART Tx as Peripheral */
					//            Config_Pin_UART_Mode(KW_TXD);

					/* Enable UART communication */
					uartClearTxQueue(UART_CH_ID_1,
							 ISR_MODE);
					uartClearRxQueue(UART_CH_ID_1,
							 ISR_MODE);

					//enable_uart(UART_CH_ID_1);
//                    UART_MidEnable(UART_CH_ID_1);
					uartMidEnable(UART_CH_ID_1);

//                    UART_ITConfig (UART_CH_ID_1,IER_THRE,ENABLE);
//                   UART_ITConfig (UART_CH_ID_1,IER_RBR,ENABLE);

					/* Configure the Receive time out for UART - RETTO */
					/* If P3 min is less than P1max then limit the time to
					   P1max as P1 max is the minimum time to determine the
					   end of frame else load P3min */
					if (l_P3MIN < l_P1MAX) {
						UART_Change_RxTimeOut
						    (UART_CH_ID_1, l_P1MAX);
						UART_Restart_RxTimeOut
						    (UART_CH_ID_1);
					}

					else {
						UART_Change_RxTimeOut
						    (UART_CH_ID_1,
						     l_P3MIN + 50000);
						UART_Restart_RxTimeOut
						    (UART_CH_ID_1);
					}

					/* Configure Tx Timeguard */
					UART_Change_TimeGuard(UART_CH_ID_1,
							      l_P4MIN);

					//AMIT
					//UART_Change_TimeGuard(UART_CH_ID_1, 1);

					/* Set the Tx length to indicate Tx bytes */
					l_TxLength =
					    l_ISO9141_14230LinkInit_S.Length;

					/* Load UART with the data bytes */
					for (fl_Cnt = 0;
					     fl_Cnt <
					     l_ISO9141_14230LinkInit_S.Length;
					     fl_Cnt++) {
						if (0 == fl_Cnt) {

							//uartMidSendFirstByte(UART_CH_ID_1,l_ISO9141_14230LinkInit_S.Data[fl_Cnt]);
							uartMidSendFirstByte
							    (UART_CH_ID_1,
							     (uint8_t *) &
							     l_ISO9141_14230LinkInit_S.
							     Data[fl_Cnt]);

							//UART_Change_TimeGuard(UART_CH_ID_1, l_P4MIN);
						}

						else {

							//uartMidSend(UART_CH_ID_1,
							//           (uint8_t *)&l_ISO9141_14230LinkInit_S.Data[fl_Cnt], 1);
						}
					}

					/* Increment l_BitPos to Receive response bytes */
					l_BitPos++;
				}

				else if (l_BitPos > WKUP_STARTREQ) {

					/* Check if ECU response has been received based on
					   RxLength information */
					if (ISO9141_14230_DelFromQ
					    (ISO9141_14230_LEN_Q, (void *)
					     &l_ISO9141_14230LinkInitRet_S.
					     Length)
					    != ISO9141_14230_Q_EMPTY) {

						/* Obtain bytes from the UART */
						for (fl_Cnt = 0;
						     fl_Cnt <
						     l_ISO9141_14230LinkInitRet_S.
						     Length; fl_Cnt++) {

							//UART_MidRecv(UART_CH_ID_1,
							//       (void*)&l_ISO9141_14230LinkInitRet_S.Data[fl_Cnt], 1);
							uartMidRecv
							    (UART_CH_ID_1,
							     (uint8_t *) &
							     l_ISO9141_14230LinkInitRet_S.
							     Data[fl_Cnt], 1);
						}

						/* Verify checksum if configured */
						if (CHECK_BITU8
						    (l_ConnectFlags,
						     BIT_CHKSUM) == FALSE) {

							/* Obtain the checksum for Init data */
							fl_ChkSum =
							    ISO9141_14230_GetChecksum
							    ((void *)
							     l_ISO9141_14230LinkInitRet_S.Data, l_ISO9141_14230LinkInitRet_S.Length, CHKSUM_RXDATA);

							/* If Checksum is ok */
							if (fl_ChkSum ==
							    CHECKSUM_OK) {

								/* Update length */
								l_ISO9141_14230LinkInitRet_S.
								    Length--;

								/* Remove the checksum byte */
								l_ISO9141_14230LinkInitRet_S.
								    Data
								    [l_ISO9141_14230LinkInitRet_S.
								     Length]
								    = 0;

								/* Set the status of Fast Init */
								l_InitStatus
								    =
								    LINKINIT_DONE;

								/* Update the Timestamp */
								ISO9141_14230_DelFromQ
								    (ISO9141_14230_TIME_Q,
								     (ISO9141_14230_Q_S
								      *)
								     &
								     l_ISO9141_14230LinkInitRet_S.
								     Timestamp);

								/* Store Protocol Id */
								l_ISO9141_14230LinkInitRet_S.
								    ProtocolId =
								    l_ProtocolId;

								/* Store Init type */
								l_ISO9141_14230LinkInitRet_S.
								    IOCtlId =
								    FAST_INIT;

								/* Report to the Call Back function */
								App_InitData
								    ((void *)
								     &l_ISO9141_14230LinkInitRet_S);
							}

							/* Report Checksum error */
							else {

								/* Set the status of Fast Init */
								l_InitStatus
								    =
								    LINKINIT_FAIL;

								/* Call application error handler */
								App_ErrHandler
								    (l_ProtocolId,
								     ECU_RESPCHKSUM_ERROR);
							}

							/* Stop Timer for Initialization for
							   ISO 9141/14230 */
							//                          stop_time_stamp(l_InitTimer);
						}

						/* If no checksum configured */
						else {

							/* Set the status of Fast Init */
							l_InitStatus =
							    LINKINIT_DONE;

							/* Update the Timestamp */
							ISO9141_14230_DelFromQ
							    (ISO9141_14230_TIME_Q,
							     (ISO9141_14230_Q_S
							      *)
							     &
							     l_ISO9141_14230LinkInitRet_S.
							     Timestamp);

							/* Store Protocol Id */
							l_ISO9141_14230LinkInitRet_S.
							    ProtocolId =
							    l_ProtocolId;

							/* Store Init type */
							l_ISO9141_14230LinkInitRet_S.
							    IOCtlId = FAST_INIT;

							/* Report to the Call Back function */
							App_InitData((void *)
								     &l_ISO9141_14230LinkInitRet_S);

							/* Stop Timer for Timestamp for
							   ISO 9141/14230 */
//                            stop_time_stamp(l_InitTimer);
					}}
				}

				else {

					/* Do nothing */
				}
			}
		}
	}

	else {

		/* Do nothing */
	}
}

/******************************************************************************
* Function name     : void ISO9141_14230_UARTIntrHandler(void)
*    returns        : void
*    arg1           : None
* Description       : This funtions is called from within the UART interrupt to
                      handle the data received by UART
* Notes             : restrictions, odd modes
*******************************************************************************/
uint8_t uartmsgcorrupt;

void ISO9141_14230_UARTIntrHandler(uint8_t p_IntrType, uint8_t uartRecvByte)
{
	uint16_t fl_Cnt;
	uint32_t fl_CurrTime;
	uint8_t fl_UARTRxByte, fl_TxByte;

	/* Update current time */
	//get_time_stamp(timestamp_id[GARUDA_KWP_CH1], &fl_CurrTime);
	//get_data_logging_time_stamp(&fl_CurrTime);
//    get_time_stamp(&fl_CurrTime);
	/* Based on the interrupt type perform operations */

	/* Rx Interrupt */
	if (CHECK_BITU8(p_IntrType, RX_INTR) != FALSE) {

		/* Check if Tx length is zero. If non zero then byte obtained is the
		   byte transmitted */
		if (l_TxLength != 0) {

			/* Determine the Tx byte from the type of Tx data */

			/* Check if Init Tx Data */
			if (l_InitStatus == LINKINIT_PENDING) {

				/* Determine the type of Init */

				/* 5 Baud Init */
				if (l_ISO9141_14230LinkInit_S.InitType ==
				    FIVE_BAUD_INIT) {

					/* Tx byte is the Keybyte 2 inverse byte */
					fl_TxByte =
					    l_ISO9141_14230LinkInitRet_S.Data[2]
					    ^ 0xFF;
				}

				else {

					/* Tx byte is the Init data byte */
					fl_TxByte =
					    l_ISO9141_14230LinkInit_S.Data
					    [l_UARTIntrTxIndex];
				}
			}

			/* Normal Tx data byte */
			else {

				/* Retrieve Tx data byte from Tx Queue */
				ISO9141_14230_GetByteFromQ
				    (ISO9141_14230_TX_Q, l_UARTIntrTxIndex,
				     &fl_TxByte);
			}

			/* Check for Corruption on the bus */

			/* Compare Received byte with Transmitted byte */
			// UART_MidRecv(UART_CH_ID_1, &fl_UARTRxByte, 1);
			uartMidRecv(UART_CH_ID_1,
				    (uint8_t *) & fl_UARTRxByte, 1);
			if (fl_UARTRxByte != fl_TxByte) {

				/* Corruption on the Bus */
				/* Abort Transmission - Flush buffers */
				uartmsgcorrupt = 1;
				uartClearTxQueue(UART_CH_ID_1, ISR_MODE);
				uartClearRxQueue(UART_CH_ID_1, ISR_MODE);

				/* Call application error handler */
				App_ErrHandler(l_ProtocolId, MSG_CORRUPTED);

				/* If Tx byte is from Initialization set status to
				   failure */
				if (l_InitStatus == LINKINIT_PENDING) {
					l_InitStatus = LINKINIT_FAIL;

					/* Stop Timer for Initialization for
					   ISO 9141/14230 */
					//                   stop_time_stamp(l_InitTimer);
				}

				/* Disable Rx Timeout */
				UART_Change_RxTimeOut(UART_CH_ID_1, 0);

				/* Reset Lengths */
				l_UARTIntrTxIndex = 0;
				l_TxLength = 0;

				/* Indicate TxComp */
				l_TxComp = !FALSE;
				l_P3MINTimeout = !FALSE;
			}

			else {

				/* Increment the Tx length to check next byte */
				l_UARTIntrTxIndex++;

				/* Check if Last byte is received */
				if (l_UARTIntrTxIndex == l_TxLength) {

					/* Configure Rx Timeout - Only when not in initialization
					   For Five Baud and Fast init timeouts are already
					   configured */
					if (l_InitStatus != LINKINIT_PENDING) {

						/* Load the timeout based on the value of P3min */
						/* If P3 min is less than P1max then limit the time to
						   P1max as P1 max is the minimum time to determine the
						   end of frame else load P3min */

/*				if (l_P3MIN < l_P1MAX) {

                            AT91C_BASE_US0->US_IER =0x00000100;
                            UART_Change_RxTimeOut(AT91C_BASE_US0, l_BaudRate,
                                                  l_P1MAX);
                            UART_Restart_RxTimeOut(AT91C_BASE_US0);

                        }
                        else
                        {

                            AT91C_BASE_US0->US_IER =0x00000100;
                            UART_Change_RxTimeOut(AT91C_BASE_US0, l_BaudRate,
                                                  l_P3MIN);
                            UART_Restart_RxTimeOut(AT91C_BASE_US0);
                        }*/
						UART_Change_RxTimeOut
						    (UART_CH_ID_1, P2_MAX_TIME);
						UART_Restart_RxTimeOut
						    (UART_CH_ID_1);

						/* Update the P3min Timeout indication in order
						   to wait for time P3min before next Txn */
						l_P3MINTimeout = FALSE;

						/* Initialize Message received status */
						l_MsgReceived = FALSE;
					}

					/* For Five Baud Mode 1 indicate Init Done */
					else if ((l_InitStatus ==
						  LINKINIT_PENDING)
						 &&
						 (l_ISO9141_14230LinkInit_S.
						  InitType == FIVE_BAUD_INIT)
						 && (l_FiveBaudMod ==
						     FIVEBAUD_MODE_ONE)) {

						/* In this Mode, receive ECU Pattern and
						   Key bytes, Transmit KeyByte inverse
						   No reception of Address Inverse byte */

						/* Set the Link Init Status to Done */
						l_InitStatus = LINKINIT_DONE;

						/* Store Protocol Id */
						l_ISO9141_14230LinkInitRet_S.
						    ProtocolId = l_ProtocolId;

						/* Store Init type */
						l_ISO9141_14230LinkInitRet_S.
						    IOCtlId = FIVE_BAUD_INIT;

						/* Store Only Key Bytes - Store the Keybytes at
						   0th and 1st element of the array and modify
						   length to indicate only Keybytes */
						l_ISO9141_14230LinkInitRet_S.
						    Data[0] =
						    l_ISO9141_14230LinkInitRet_S.
						    Data[1];
						l_ISO9141_14230LinkInitRet_S.
						    Data[1] =
						    l_ISO9141_14230LinkInitRet_S.
						    Data[2];
						l_ISO9141_14230LinkInitRet_S.
						    Length = 2;

						/* Report Link init done to the Call Back
						   function */
						App_InitData((void *)
							     &l_ISO9141_14230LinkInitRet_S);

						/* Stop Timer for Timestamp for
						   ISO 9141/14230 */
						//                       stop_time_stamp(l_InitTimer);

						/* Indicate response bytes received */
						l_MsgReceived = !FALSE;

						/* Load P3 min timeout for minimum time before
						   Data transmission */
						UART_Change_RxTimeOut
						    (UART_CH_ID_1, l_P3MIN);
						UART_Restart_RxTimeOut
						    (UART_CH_ID_1);
						l_P3MINTimeout = FALSE;
					}

					else {

						/* No operation */
						l_TxComp = l_TxComp;
					}

					/* Last Tx byte received - Reset Lengths */
					l_UARTIntrTxIndex = 0;
					l_TxLength = 0;

					/* Indicate Tx Completion */
					l_TxComp = !FALSE;
				}
			}
		}

		else {
			/* Check if the UART RX interrupt is due to Link initialization */
			if (l_InitStatus == LINKINIT_PENDING) {

				/* 5 Baud Init */
				if (l_ISO9141_14230LinkInit_S.InitType ==
				    FIVE_BAUD_INIT) {

					/* Determine the byte received - Load corresponding
					   timeouts */
					if (l_RxLength == PATTERN_BYTE_POS) {

						/* Received the pattern byte - Increment the length */
						l_RxLength++;

						/* Load the Receive timeout W2 - RETTO */
						UART_Change_RxTimeOut
						    (UART_CH_ID_1, l_W2);
						UART_Restart_RxTimeOut
						    (UART_CH_ID_1);
					}

					else if (l_RxLength == KEY_BYTE1_POS) {

						/* Received the Key byte 1 - Increment the length */
						l_RxLength++;

						/* Load the Receive timeout W3 - RETTO */
						UART_Change_RxTimeOut
						    (UART_CH_ID_1, l_W3);
						UART_Restart_RxTimeOut
						    (UART_CH_ID_1);
					}

					else if (l_RxLength == KEY_BYTE2_POS) {

						/* Received the Key byte 2 - Increment the length */
						l_RxLength++;

						/* Update the length information to the Length Queue */
						ISO9141_14230_AddToQ
						    (ISO9141_14230_LEN_Q,
						     (const ISO9141_14230_Q_S
						      *)&l_RxLength, 0);

						/* Disable Rx Timeout - For Address inverse byte
						   Rx timeout is loaded based on the 5 Baud mode */
						UART_Change_RxTimeOut
						    (UART_CH_ID_1, 0);
					}

					else if (l_RxLength ==
						 ADDRESS_BYTE_INV_POS) {

						/* Received the Address byte inverse - Increment the
						   length */
						l_RxLength++;

						/* Update the length information to the Length Queue */
						ISO9141_14230_AddToQ
						    (ISO9141_14230_LEN_Q,
						     (const ISO9141_14230_Q_S
						      *)&l_RxLength, 0);

						/* Reset Rx Length */
						l_RxLength = 0;

						/* Disable Rx Timeout */
						UART_Change_RxTimeOut
						    (UART_CH_ID_1, 0);
					}

					else {

						/* Do nothing */
					}
				}

				/* Fast Init */
				else {

					/* If First byte received */
					if (l_RxLength == 0) {

						/* Load the RxTimeout to determine end of frame */
						UART_Change_RxTimeOut
						    (UART_CH_ID_1, l_P1MAX);
						UART_Restart_RxTimeOut
						    (UART_CH_ID_1);

						/* FIRST_BYTE_RECVD indication ???? */
						/* Call back function */
						/* Commented the First Byte Rx Indication for Fast Init */
						/* RA Consultant recomended code */
						// App_FirstByteRxd(l_ProtocolId,fl_CurrTime);

						/* Update Rx timestamp */
						ISO9141_14230_AddToQ
						    (ISO9141_14230_TIME_Q,
						     (const ISO9141_14230_Q_S
						      *)&fl_CurrTime, 0);

						/* Initialize Message received status */
						l_MsgReceived = FALSE;
					}

					/* Received ECU response byte - Increment length */
					l_RxLength++;
				}
			}

			/* Receive ECU response bytes for Tester request */
			else if ((l_InitStatus == LINKINIT_DONE) ||
				 (l_InitStatus == NO_LINKINIT) ||
				 (l_InitStatus == LINKINIT_FAIL)) {

				/* If First byte received */
				if (l_RxLength == 0) {
					l_LengthByte = 0;
					l_PktLength = 0;
					if (KWP_PROTOCOL_ID ==
					    get_ISO9141_or_14230()) {
						l_PktLength =
						    ISO9141_14230_Get_Data_Length
						    (uartRecvByte,
						     &l_LengthByte);
					}

					/* Load the RxTimeout to determine end of frame */
					UART_Change_RxTimeOut(UART_CH_ID_1,
							      l_P1MAX);
					UART_Restart_RxTimeOut(UART_CH_ID_1);

					/* FIRST_BYTE_RECVD indication ???? */
					/* Call back function */
					App_FirstByteRxd(l_ProtocolId,
							 fl_CurrTime);

					/* Initialize Message received status */
					l_MsgReceived = FALSE;
				}

				else if (l_RxLength == l_LengthByte) {
					if (KWP_PROTOCOL_ID ==
					    get_ISO9141_or_14230()) {
						l_PktLength =
						    ISO9141_14230_Get_Data_Length
						    (uartRecvByte,
						     &l_LengthByte);
					}
				}

				else {

					//Do nothing    
				}

				/* Received ECU response byte - Increment length */
				l_RxLength++;
				if (l_RxLength == l_PktLength) {

					// Complete packet received
					// Handle the timeout functionalities
					l_PktLength = 0;
					fl_CurrTime = fl_CurrTime - (l_P1MAX);

					/* Update Rx timestamp */
					ISO9141_14230_AddToQ
					    (ISO9141_14230_TIME_Q,
					     (const ISO9141_14230_Q_S *)
					     &fl_CurrTime, 0);

					/* Update the length information to the Length Queue */
					ISO9141_14230_AddToQ
					    (ISO9141_14230_LEN_Q,
					     (const ISO9141_14230_Q_S *)
					     &l_RxLength, 0);

					/* Reset Rx Length */
					l_RxLength = 0;

					/* Indicate Message received */
					l_MsgReceived = !FALSE;

					/* Check P3min to load additional timeout starting from
					   P1max */
					/* If P3 min is less than P1max then next Tx could be
					   started immediately else wait for P3min - P1max */
					if (l_P3MIN < l_P1MAX) {

						/* Update the status of P3min timeout */
						l_P3MINTimeout = !FALSE;

						/* Disable Rx timeout */
//                        UART_Change_RxTimeOut(UART_CH_ID_1, 0);
					}

					else {

						/* - This was an Error which we had face in REVA for Diagnostic Tester Application
						   - We had Changed the implementation of KWP Packet handling based on the Length.
						   - So we were not maintaining the delay of 55 mses for response to request (P3_Min) timeout.
						   - We shouldn't subtract the l_P1MAX timeout from l_P3MIN, (55-20) = 35 msecs delay, which is wrong. 
						 */
						UART_Change_RxTimeOut(UART_CH_ID_1, l_P3MIN);	/* FIX FOR REVA, -l_P1MAX removed. */
						UART_Restart_RxTimeOut
						    (UART_CH_ID_1);
					}
				}
			}

			else {

				/* If there is Link Init failure do nothing */
			}
		}
	}

	/* Tx interrupt */
	if (CHECK_BITU8(p_IntrType, TX_INTR) != FALSE) {
	}
	uint8_t txData;

	/* Rx Timeout interrupt */
	if (CHECK_BITU8(p_IntrType, RXTIMEOUT_INTR) != FALSE) {

		/* Check if the UART interrupt is due to Link initialization */
		if (l_InitStatus == LINKINIT_PENDING) {

			/* 5 Baud Init */
			if (l_ISO9141_14230LinkInit_S.InitType ==
			    FIVE_BAUD_INIT) {

				/* Timeout due to W4 wait before Keybyte 2 inverse Tx */
				if ((l_RxLength == ADDRESS_BYTE_INV_POS)
				    && (l_TxLength != 0)) {

					/* W4 Timeout to transmit Key byte 2 inverse */

					/* Send Key byte 2 inverse */
					txData =
					    (l_ISO9141_14230LinkInitRet_S.Data
					     [2] ^ 0xFF);

					//uartMidSend(UART_CH_ID_1, &txData, 1);
					uartMidSend(UART_CH_ID_1,
						    (uint8_t *) &
						    l_ISO9141_14230LinkInitRet_S.
						    Data[fl_Cnt], 1);

					/* Timeout applicable only to Mode zero */
					if (l_FiveBaudMod == FIVEBAUD_MODE_ZERO) {

						/* Load Max Rx timeout for Address inverse byte
						   W4 in - RETTO */
						UART_Change_RxTimeOut
						    (UART_CH_ID_1, W4_MAX_TIME);
						UART_Restart_RxTimeOut
						    (UART_CH_ID_1);
					}
				}

				/* Timeout on Pattern Byte ?????? */
				else if (l_RxLength == PATTERN_BYTE_POS) {

					/* Call application error handler */
					App_ErrHandler(l_ProtocolId,
						       PATTERN_BYTE_TIMEOUT);

					/* Disable Rx timeout */
					UART_Change_RxTimeOut(UART_CH_ID_1, 0);

					/* Reset Rx Length */
					l_RxLength = 0;
					l_TxComp = !FALSE;
					l_P3MINTimeout = !FALSE;

					/* Set Link Init failure */
					l_InitStatus = LINKINIT_FAIL;

					/* Stop Timer for Initialization for
					   ISO 9141/14230 */
					//                   stop_time_stamp(l_InitTimer);
				}

				/* Timeout on Key Byte 1 ??????? */
				else if (l_RxLength == KEY_BYTE1_POS) {

					/* Call application error handler */
					App_ErrHandler(l_ProtocolId,
						       KEY_BYTE1_TIMEOUT);
					l_TxComp = !FALSE;

					/* Disable Rx timeout */
					UART_Change_RxTimeOut(UART_CH_ID_1, 0);

					/* Reset Rx Length */
					l_RxLength = 0;
					l_P3MINTimeout = !FALSE;

					/* Set Link Init failure */
					l_InitStatus = LINKINIT_FAIL;

					/* Stop Timer for Initialization for
					   ISO 9141/14230 */
//                   stop_time_stamp(l_InitTimer);
				}

				/* Timeout on Key Byte 2 ??????? */
				else if (l_RxLength == KEY_BYTE2_POS) {

					/* Call application error handler */
					App_ErrHandler(l_ProtocolId,
						       KEY_BYTE2_TIMEOUT);
					l_TxComp = !FALSE;
					l_P3MINTimeout = !FALSE;

					/* Disable Rx timeout */
					UART_Change_RxTimeOut(UART_CH_ID_1, 0);

					/* Reset Rx Length */
					l_RxLength = 0;

					/* Set Link Init failure */
					l_InitStatus = LINKINIT_FAIL;

					/* Stop Timer for Initialization for
					   ISO 9141/14230 */
					//                  stop_time_stamp(l_InitTimer);
				}

				/* Timeout on Address Byte Inverse ??????? */

				else if (l_RxLength == ADDRESS_BYTE_INV_POS) {

					/* Call application error handler */
					App_ErrHandler(l_ProtocolId,
						       ADDRESS_BYTE_TIMEOUT);
					l_TxComp = !FALSE;
					l_P3MINTimeout = !FALSE;

					/* Disable Rx timeout */
					UART_Change_RxTimeOut(UART_CH_ID_1, 0);

					/* Reset Rx Length */
					l_RxLength = 0;

					/* Set Link Init failure */
					l_InitStatus = LINKINIT_FAIL;

					/* Stop Timer for Initialization for
					   ISO 9141/14230 */
					//                   stop_time_stamp(l_InitTimer);
				}

				else {

					/* Invalid interrupt indication */
				}
			}

			/* Fast Init */
			else {

				/* Check if length is nonzero. If no bytes are received then
				   timeout on Fast Init ECU response else indicate end of
				   Fast Init ECU response */
				if (l_RxLength != 0) {

					/* End of frame indication */

					/* Update the length information to the Length Queue */
					ISO9141_14230_AddToQ
					    (ISO9141_14230_LEN_Q,
					     (const ISO9141_14230_Q_S *)
					     &l_RxLength, 0);

					/* Reset Rx Length */
					l_RxLength = 0;

					/* Indicate Message received */
					l_MsgReceived = !FALSE;

					/* If P3 min is less than P1max then next Tx could be
					   started immediately else wait for P3min - P1max */
					if (l_P3MIN < l_P1MAX) {

						/* Update the status of P3min timeout */
						l_P3MINTimeout = !FALSE;
					}

					else {
						UART_Change_RxTimeOut
						    (UART_CH_ID_1,
						     (l_P3MIN - l_P1MAX));
						UART_Restart_RxTimeOut
						    (UART_CH_ID_1);
					}
				}

				else {

					/* Timeout on Fast Init ECU reponse */
					/* Call application error handler */
					App_ErrHandler(l_ProtocolId,
						       FASTINIT_RESP_TIMEOUT);

					/* Set Link Init failure */
					l_InitStatus = LINKINIT_FAIL;

					/* Stop Timer for Initialization for
					   ISO 9141/14230 */
					//                  stop_time_stamp(l_InitTimer);

					/* Disable Rx timeout */
					UART_Change_RxTimeOut(UART_CH_ID_1, 0);

					/* Update the status of P3min timeout - Could load the Tx
					   bytes */
					l_P3MINTimeout = !FALSE;
				}
			}
		}

		/* Normal operation - Timeout / End of ECU response bytes */
		else if ((l_InitStatus == LINKINIT_DONE) ||
			 (l_InitStatus == NO_LINKINIT) ||
			 (l_InitStatus == LINKINIT_FAIL)) {

			/* Check the status of frame received to differentiate between
			   timeout and last ECU byte */
			if (l_MsgReceived == FALSE) {	/* Check if length is nonzero. If no bytes are received then
							   timeout on ECU response else indicate end of ECU response */

				/* Note : Tata Motors Issue Identified June 28-2016 
				   1. Some ECUs were not behaving as per the standards, and they 
				   had interbye timeout of 65mesecs, (Idealy-1-2msecs).

				   2. This caused the loss of the remainaing bytes after 65 msecs, 
				   since the Algorithm in Garuda Firmware was Timeout based.

				   3. Basicslly P1Max timeout would occur and packets would be lost,
				   So validation based on packet length was initiated.

				   4. Check for the reception of complete frame based on the length, 
				   and if complete frame not received then update the P1max timeout 
				   continously untill compete data packet is received 
				 */
				if (l_RxLength != 0) {
					if (KWP_PROTOCOL_ID ==
					    get_ISO9141_or_14230()) {
						if (l_RxLength != l_PktLength) {
							l_P1MAX_ExtendedTimeout
							    += l_P1MAX;
							if (l_P1MAX_ExtendedTimeout < P1_MAX_EXTENDED_TIMEOUT) {

								/* Load the RxTimeout to determine end of frame */
								UART_Change_RxTimeOut
								    (UART_CH_ID_1,
								     l_P1MAX);
								UART_Restart_RxTimeOut
								    (UART_CH_ID_1);
								return;
							}

							else {

								//P1_MAX_WAIT_TIMEOUT has occoured, continue with the normal flow
							}
						}
					}

					/* End of frame indication */
					l_P1MAX_ExtendedTimeout = 0;

					/* Subtract time spent for P1 MAX */
					fl_CurrTime = fl_CurrTime - (l_P1MAX);

					/* Update Rx timestamp */
					ISO9141_14230_AddToQ
					    (ISO9141_14230_TIME_Q,
					     (const ISO9141_14230_Q_S *)
					     &fl_CurrTime, 0);

					/* Update the length information to the Length Queue */
					ISO9141_14230_AddToQ
					    (ISO9141_14230_LEN_Q,
					     (const ISO9141_14230_Q_S *)
					     &l_RxLength, 0);

					/* Reset Rx Length */
					l_RxLength = 0;

					/* Indicate Message received */
					l_MsgReceived = !FALSE;

					/* Check P3min to load additional timeout starting from
					   P1max */

					/* If P3 min is less than P1max then next Tx could be
					   started immediately else wait for P3min - P1max */
					if (l_P3MIN < l_P1MAX) {

						/* Update the status of P3min timeout */
						l_P3MINTimeout = !FALSE;

						/* Disable Rx timeout */
						UART_Change_RxTimeOut
						    (UART_CH_ID_1, 0);
					}

					else {
						UART_Change_RxTimeOut
						    (UART_CH_ID_1,
						     (l_P3MIN - l_P1MAX));
						UART_Restart_RxTimeOut
						    (UART_CH_ID_1);
					}
				}

				else {

					/* Timeout on ECU reponse for Tester request */
					/* Call application error handler */
					App_ErrHandler(l_ProtocolId,
						       ECU_RESP_TIMEOUT);

					/* Minimum time P3min has elapsed. Next Tx can be started */
					l_P3MINTimeout = !FALSE;

					/* Disable Rx timeout */
					UART_Change_RxTimeOut(UART_CH_ID_1, 0);
				}
			}

			else {

				/* Minimum time P3min has elapsed. Next Tx can be started */
				l_P3MINTimeout = !FALSE;

				/* Disable Rx timeout */
				UART_Change_RxTimeOut(UART_CH_ID_1, 0);
			}
		}

		else {

			/* If there is Link Init failure report ECU timeout */

			/* Timeout on ECU reponse for Tester request */
			/* Call application error handler */
			App_ErrHandler(l_ProtocolId, ECU_RESP_TIMEOUT);

			/* Minimum time P3min has elapsed. Next Tx can be started */
			l_P3MINTimeout = !FALSE;

			/* Disable Rx timeout */
			UART_Change_RxTimeOut(UART_CH_ID_1, 0);
		}
	}
}

/******************************************************************************
* Function name     : void ISO9141_14230_Reset(void)
*    returns        : void
*    arg1           : None
* Description    uint8_t   : This funtions performs the reset of communication
* Notes             : restrictions, odd modes
*******************************************************************************/
void ISO9141_14230_Reset(void)
{
	/* Reset UART communication */
	//UART_DeInit(UART_CH_ID_1);
	UART_MidDeInit(UART_CH_ID_1);

	/* Initialize Transmission Queue */
	memset((void *)&l_ISO9141_14230_TxQ, 0,
	       (sizeof(uint8_t) * ISO9141_14230_MAXTXQSIZE));

	/* Initialize Reception Queue */
	memset((void *)&l_ISO9141_14230_RxQ, 0,
	       (sizeof(uint8_t) * ISO9141_14230_MAXRXQSIZE));

	/* Initialize Rx Length Queue */
	memset((void *)&l_ISO9141_14230_RxLenQ, 0,
	       (sizeof(uint8_t) * ISO9141_14230_MAXRXLENQSIZE));

	/* Initialize Rx Time Queue */
	memset((void *)&l_ISO9141_14230_RxTimeQ, 0,
	       (sizeof(uint8_t) * ISO9141_14230_MAXRXTIMEQSIZE));

	/* Initializuint8_te Link Init structure */
	memset((void *)&l_ISO9141_14230LinkInit_S, 0,
	       sizeof(ISO9141_14230_LinkInit_S));

	/* Initialize Init Return data structure */
	memset((void *)&l_ISO9141_14230LinkInitRet_S, 0,
	       sizeof(ISO9141_14230_LinkInitRet_S));

	/* Stop Timer for Timestamp for ISO 9141 / 14230 */
//    stop_time_stamp(timestamp_id[GARUDA_KWP_CH1]);

	/* Stop Timer for Timestamp for
	   ISO 9141/14230 */
//    stop_time_stamp(l_InitTimer);

	/* Reset the Front and Rear variables */
	l_TxQFront = 0;
	l_TxQRear = 0;
	l_RxQFront = 0;
	l_RxQRear = 0;
	l_RxLenQFront = 0;
	l_RxLenQRear = 0;
	l_RxTimeQFront = 0;
	l_RxTimeQRear = 0;

	/* Reset all the ISO 9141 / 14230 parameters to default values */
	l_Parity = NO_PARITY;
	l_DataBits = EIGHT_BITS;
	l_ConnectFlags = 0;
	l_loopback = 0;
	l_P1MAX = 20000;
	l_P3MIN = 55000;
	l_P4MIN = 10000;
	l_W0 = 300000;
	l_W1 = 300000;
	l_W2 = 20000;
	l_W3 = 20000;
	l_W4 = 49500;		/* Minimum Time of 50ms is not possible
				   because of the delay in trasnmission
				   and the max time being 50ms as well */
	l_W5 = 300000;
	l_TIDLE = 300000;
	l_TINIL = 25000;
	l_TWUP = 50000;
	l_FiveBaudMod = FIVEBAUD_MODE_ZERO;
	l_RxLength = 0;
	l_TxLength = 0;
	l_TxIndex = 0;
	l_BitPos = 0;
	l_InitStatus = NO_LINKINIT;
	l_RefIdleTime = 0;
	l_RefInitTime = 0;
	l_UARTIntrTxIndex = 0;
	l_P3MINTimeout = !FALSE;
	l_MsgReceived = FALSE;
	l_TxComp = FALSE;

	/*DisConnect K & L from OBD Connector */
//    Set_Pin_Low(LLINE_ISO_SLCT);
	//   Set_Pin_Low(KLINE_ISO_SLCT);
}

/******************************************************************************
* Function name     : void ISO9141_14230_TxTask(void)
*    returns        : void
*    arg1           : None
* Description       : This funtion loads a Tx frame from TxQ for transmission
                      over UART
* Notes             : restrictions, odd modes
*******************************************************************************/
void ISO9141_14230_TxTask(void)
{
	Mid_API_Status_t fl_UARTStatus = MID_PASS;
	ISO9141_14230_TxMsg_S fl_ISO9141_14230Tx_S;
	ISO9141_14230_QSTATUS fl_TxQStatus;
	ISO9141_14230_QSTATUS fl_RxQStatus;

	/* Check if transmission completed during non-init mdoe to remove the Tx
	   frame from Queue */
	if ((l_InitStatus != LINKINIT_PENDING) && (l_TxComp != FALSE)) {

		/* If all the bytes are transmitted then remove the frame from
		   Queue */
		ISO9141_14230_DelFromQ(ISO9141_14230_TX_Q,
				       (ISO9141_14230_Q_S *) &
				       fl_ISO9141_14230Tx_S);
		if (l_loopback) {
			fl_ISO9141_14230Tx_S.Flags = 0x00000001;

			//get_data_logging_time_stamp(&fl_ISO9141_14230Tx_S.Timestamp);
//          get_time_stamp(&fl_ISO9141_14230Tx_S.Timestamp);
			/* Store the message onto the Rx queue */
			fl_RxQStatus =
			    ISO9141_14230_AddToQ(ISO9141_14230_RX_Q,
						 (const ISO9141_14230_Q_S *)
						 &fl_ISO9141_14230Tx_S, 0);

			/* Report error if RxQ full */
			if (ISO9141_14230_Q_FULL == fl_RxQStatus) {

				/* Set Error Code and report to Call back function */
				App_ErrHandler(l_ProtocolId, RXQ_OVERFLOW);
			}
		}
		l_TxComp = FALSE;
	}

	/* Read a frame from Queue if

	   (the initialization is not in the pending state AND

	   (the minimum time between ECU response and
	   tester request has elapsed AND the previous frame is loaded to UART
	   for transmission

	   OR

	   the Tx frame length is nonzero AND Index has not reached the Tx frame
	   length) ) */
	if (((l_InitStatus != LINKINIT_PENDING) &&
	     (((l_P3MINTimeout != FALSE) && (l_TxLength == 0))
	      ||
	      ((l_TxLength != 0) && (l_TxIndex < l_TxLength)))) &&
	    (l_RxLength == 0))
//
//    if( (l_InitStatus != LINKINIT_PENDING) &&
//
//        ( ( (l_P3MINTimeout != FALSE) && (l_TxLength == 0) )
//
//        ||
//
//          ( (l_TxLength != 0) && (l_TxIndex < l_TxLength)) )
//      )
	{

		/* Initialize Index for Tx bytes when a new frame is to be
		   transmitted */
		if ((l_TxLength == 0) && (l_TxIndex != 0)) {

			/* New frame - Restart Index */
			l_TxIndex = 0;
		}

		/* Obtain the new message to be transmitted from the queue */
		fl_TxQStatus =
		    ISO9141_14230_ReadFromQ(ISO9141_14230_TX_Q,
					    (ISO9141_14230_Q_S *) &
					    fl_ISO9141_14230Tx_S);

		/* Check if the queue is empty */
		if (ISO9141_14230_Q_EMPTY == fl_TxQStatus) {

			/* Exit the task - No data to transmit */
		}

		/* Load the Tx bytes onto UART */
		else {

			/* Save the Tx frame length and Load Tx Interbyte time for the new
			   frame */
			if (l_TxIndex == 0) {

				/* Save Tx Length */
				l_TxLength = fl_ISO9141_14230Tx_S.Length;

				/* Configure Tx Timeguard */
				UART_Change_TimeGuard(UART_CH_ID_1, l_P4MIN);
			}

			/* Start loading Tx data onto the UART */
			while (l_TxIndex < fl_ISO9141_14230Tx_S.Length) {

				/* Load to UART Tx */
				fl_UARTStatus =
				    uartMidSend(UART_CH_ID_1,
						&fl_ISO9141_14230Tx_S.Data
						[l_TxIndex], 1);

				/* If UART Queue is full then stop loading */
				if (fl_UARTStatus == MID_FAIL) {

					/* Exit Loading further bytes */
					break;
				}

				/* Increment Index */
				l_TxIndex++;
			}
		}
	}
}

/******************************************************************************
* Function name     : void ISO9141_14230_RxTask(void)
*    returns        : void
*    arg1           : None
* Description       : This funtion obtains Rx data from the UART buffer and
                      updates the Receive queue
* Notes             : restrictions, odd modes
*******************************************************************************/
void ISO9141_14230_RxTask(void)
{
	ISO9141_14230_RxMsg_S fl_ISO9141_14230Rx_S;
	uint16_t fl_Cnt;
	uint8_t fl_ChkSum;
	uint16_t l_Chk78Length = 0, l_Chk78Frame = 0, l_Chk78Index = 0;
	bool l_Rxd78 = FALSE;

	ISO9141_14230_QSTATUS fl_RxQStatus;

	/* Poll the Length Queue only if not in Fast or Five Baud Initialization */
	if (l_InitStatus != LINKINIT_PENDING) {

		/* Read the length queue to check for receive data and thereby to obtain
		   the length of the Received frame */
		fl_RxQStatus =
		    ISO9141_14230_DelFromQ(ISO9141_14230_LEN_Q,
					   (ISO9141_14230_Q_S *) &
					   fl_ISO9141_14230Rx_S.Length);

		/* Check if the queue is empty */
		if (ISO9141_14230_Q_EMPTY == fl_RxQStatus) {

			/* Exit the task - No data to receive */
		}

		else {

			/* Obtain Rx data upto the frame length from the UART */
			for (fl_Cnt = 0;
			     fl_Cnt < fl_ISO9141_14230Rx_S.Length; fl_Cnt++) {

				//UART_MidRecv(UART_CH_ID_1,
				//                   &fl_ISO9141_14230Rx_S.Data[fl_Cnt], 1);
				uartMidRecv(UART_CH_ID_1,
					    &fl_ISO9141_14230Rx_S.Data[fl_Cnt],
					    1);
			}

			/* Update the Rx flags */
			fl_ISO9141_14230Rx_S.Flags = 0;

			/* Verify checksum if configured */
			if (CHECK_BITU8(l_ConnectFlags, BIT_CHKSUM) == FALSE) {

				/* Obtain the checksum for Rx data */
				fl_ChkSum =
				    ISO9141_14230_GetChecksum
				    (fl_ISO9141_14230Rx_S.Data,
				     fl_ISO9141_14230Rx_S.Length,
				     CHKSUM_RXDATA);

				/* If Checksum is ok */
				if (fl_ChkSum == CHECKSUM_OK) {

					/* Update length */
					fl_ISO9141_14230Rx_S.Length--;

					/* Remove the checksum byte - Update only data */
					fl_ISO9141_14230Rx_S.Data
					    [fl_ISO9141_14230Rx_S.Length] = 0;

					/* Update the Rx timestamp to the Rx frame */
					ISO9141_14230_DelFromQ
					    (ISO9141_14230_TIME_Q,
					     (ISO9141_14230_Q_S *) &
					     fl_ISO9141_14230Rx_S.Timestamp);

					/* Check if the message qualifies for the filter */
					if (J2534_checkFilter
					    (&fl_ISO9141_14230Rx_S.Data[0],
					     fl_ISO9141_14230Rx_S.Length,
					     GARUDA_KWP_CH1) == J2534_PASS) {

						/* Store the message onto the Rx queue */
						fl_RxQStatus =
						    ISO9141_14230_AddToQ
						    (ISO9141_14230_RX_Q,
						     (const ISO9141_14230_Q_S *)
						     &fl_ISO9141_14230Rx_S, 0);

						/* Report error if RxQ full */
						if (ISO9141_14230_Q_FULL ==
						    fl_RxQStatus) {

							/* Set Error Code and report to Call back function */
							App_ErrHandler
							    (l_ProtocolId,
							     RXQ_OVERFLOW);
						}
					}
				}

				/* Report Checksum error */
				else {

					/* For 14230 check for 78 responses */
					if (l_ProtocolId == KWP_PROTOCOL_ID) {

						/* Save the total length */
						l_Chk78Length =
						    fl_ISO9141_14230Rx_S.Length;
						l_Chk78Frame = 0;

						/* Check for the response $78 */
						for (l_Chk78Index = 0;
						     l_Chk78Index <
						     l_Chk78Length;
						     l_Chk78Index++) {

							/* Search if 7F and 78 are in the response */
							if ((fl_ISO9141_14230Rx_S.Data[l_Chk78Index] == 0x7F) && (fl_ISO9141_14230Rx_S.Data[l_Chk78Index + 2] == 0x78)) {

								/* Received 78 response */
								l_Rxd78 =
								    !FALSE;

								/* Verify checksum for the 78 response frame */

								/* Obtain the checksum for Rx data */
								fl_ChkSum =
								    ISO9141_14230_GetChecksum
								    (&fl_ISO9141_14230Rx_S.
								     Data
								     [l_Chk78Frame],
								     (l_Chk78Index
								      + 4 -
								      l_Chk78Frame),
								     CHKSUM_RXDATA);

								/* If Checksum is ok */
								if (fl_ChkSum
								    ==
								    CHECKSUM_OK)
								{

									/* Update length */
									fl_ISO9141_14230Rx_S.
									    Length
									    =
									    l_Chk78Index
									    +
									    3 -
									    l_Chk78Frame;

									/* Update the Rx timestamp to the Rx frame */
									/*get_time_stamp(timestamp_id[GARUDA_KWP_CH1],
									   &fl_ISO9141_14230Rx_S.Timestamp); */
									//get_data_logging_time_stamp(&fl_ISO9141_14230Rx_S.Timestamp);
//                                   get_time_stamp(&fl_ISO9141_14230Rx_S.Timestamp);
									/* Check if the message qualifies for the
									   filter */
									if (J2534_checkFilter(&fl_ISO9141_14230Rx_S.Data[l_Chk78Frame], fl_ISO9141_14230Rx_S.Length, GARUDA_KWP_CH1)
									    ==
									    J2534_PASS)
									{

										/* Store the message onto the Rx queue */
										fl_RxQStatus
										    =
										    ISO9141_14230_AddToQ
										    (ISO9141_14230_RX_Q,
										     (const
										      ISO9141_14230_Q_S
										      *)
										     &fl_ISO9141_14230Rx_S, l_Chk78Frame);

										/* Report error if RxQ full */
										if (ISO9141_14230_Q_FULL == fl_RxQStatus) {

											/* Set Error Code and report to Call
											   back function */
											App_ErrHandler
											    (l_ProtocolId,
											     RXQ_OVERFLOW);
											break;
										}
									}

									/* Update the index */
									l_Chk78Index
									    =
									    l_Chk78Index
									    + 4;
									l_Chk78Frame
									    =
									    l_Chk78Frame
									    +
									    fl_ISO9141_14230Rx_S.
									    Length
									    + 1;
								}

								else {

									/* Call application error handler */
									App_ErrHandler
									    (l_ProtocolId,
									     ECU_RESPCHKSUM_ERROR);
									break;
								}
							}
						}

						/* Check for positive response after 78 */
						if ((l_Rxd78 != FALSE)
						    && (l_Chk78Frame <
							l_Chk78Length)) {

							/* Positive reponse received */

							/* Obtain the checksum for Rx data */
							fl_ChkSum =
							    ISO9141_14230_GetChecksum
							    (&fl_ISO9141_14230Rx_S.
							     Data[l_Chk78Frame],
							     (l_Chk78Index -
							      l_Chk78Frame),
							     CHKSUM_RXDATA);

							/* If Checksum is ok */
							if (fl_ChkSum ==
							    CHECKSUM_OK) {

								/* Update length */
								fl_ISO9141_14230Rx_S.
								    Length =
								    (l_Chk78Index
								     -
								     l_Chk78Frame
								     - 1);

								/* Update the Rx timestamp to the Rx frame */
								/*get_time_stamp(timestamp_id[GARUDA_KWP_CH1],
								   &fl_ISO9141_14230Rx_S.Timestamp); */
								//get_data_logging_time_stamp(&fl_ISO9141_14230Rx_S.Timestamp);
//                                get_time_stamp(&fl_ISO9141_14230Rx_S.Timestamp);
								/* Check if the message qualifies for the
								   filter */
								if (J2534_checkFilter(&fl_ISO9141_14230Rx_S.Data[l_Chk78Frame], fl_ISO9141_14230Rx_S.Length, GARUDA_KWP_CH1)
								    ==
								    J2534_PASS)
								{

									/* Store the message onto the Rx queue */
									fl_RxQStatus
									    =
									    ISO9141_14230_AddToQ
									    (ISO9141_14230_RX_Q,
									     (const
									      ISO9141_14230_Q_S
									      *)
									     &fl_ISO9141_14230Rx_S, l_Chk78Frame);

									/* Report error if RxQ full */
									if (ISO9141_14230_Q_FULL == fl_RxQStatus) {

										/* Set Error Code and report to Call
										   back function */
										App_ErrHandler
										    (l_ProtocolId,
										     RXQ_OVERFLOW);
									}
								}
							}

							else {

								/* Call application error handler */
								App_ErrHandler
								    (l_ProtocolId,
								     ECU_RESPCHKSUM_ERROR);
							}
						}

						/* If no 78 detected report error */
						if (l_Chk78Frame == 0) {

							/* Call application error handler */
							App_ErrHandler
							    (l_ProtocolId,
							     ECU_RESPCHKSUM_ERROR);
						}
					}

					else {

						/* Call application error handler */
						App_ErrHandler(l_ProtocolId,
							       ECU_RESPCHKSUM_ERROR);
					}
				}
			}

			/* If no checksum configured */
			else {

				/* Update the Rx timestamp to the Rx frame */
				ISO9141_14230_DelFromQ(ISO9141_14230_TIME_Q,
						       (ISO9141_14230_Q_S
							*) &
						       fl_ISO9141_14230Rx_S.
						       Timestamp);

				/* Check if the message qualifies for the filter */
				if (J2534_checkFilter
				    (&fl_ISO9141_14230Rx_S.Data[0],
				     fl_ISO9141_14230Rx_S.Length,
				     GARUDA_KWP_CH1) == J2534_PASS) {

					/* Store the message onto the Rx queue */
					fl_RxQStatus =
					    ISO9141_14230_AddToQ
					    (ISO9141_14230_RX_Q,
					     (const ISO9141_14230_Q_S *)
					     &fl_ISO9141_14230Rx_S, 0);

					/* Report error if RxQ full */
					if (ISO9141_14230_Q_FULL ==
					    fl_RxQStatus) {

						/* Set Error Code and report to Call back function */
						App_ErrHandler(l_ProtocolId,
							       RXQ_OVERFLOW);
					}
				}
			}
		}
	}
}

/******************************************************************************
* Function name     : ISO9141_14230_RETCODE
                              ISO9141_14230_WriteMsg(ISO9141_14230_TxMsg_S
                                                      *p_ISO9141_14230TxMsg_SP)
*    returns        : ISO9141_14230_RETCODE: Returns ISO9141_14230_TXQ_FULL if
                      TxQ is full
*    arg1           : ISO9141_14230_TxMsg_S *p_ISO9141_14230TxMsg_SP
* Description       : This funtion loads a frame onto the TxQ for transmission
* Notes             : restrictions, odd modes
*******************************************************************************/
ISO9141_14230_RETCODE ISO9141_14230_WriteMsg(ISO9141_14230_TxMsg_S
					     * p_ISO9141_14230TxMsg_SP)
{
	ISO9141_14230_RETCODE fl_RetCode = NO_ERROR;
	ISO9141_14230_QSTATUS fl_TxQStatus;
	uint8_t fl_ChkSum;

	/* Calculate checksum if configured */
	if (CHECK_BITU8(l_ConnectFlags, BIT_CHKSUM) == FALSE) {

		/* Obtain the checksum for Tx data */
		fl_ChkSum =
		    ISO9141_14230_GetChecksum(p_ISO9141_14230TxMsg_SP->Data,
					      p_ISO9141_14230TxMsg_SP->Length,
					      CHKSUM_TXDATA);

		/* Store the checksum as the last byte */
		p_ISO9141_14230TxMsg_SP->Data[p_ISO9141_14230TxMsg_SP->Length] =
		    fl_ChkSum;

		/* Update the length for the checksum */
		p_ISO9141_14230TxMsg_SP->Length++;
	}

	/* Add the message to be transmitted to the Queue */
	fl_TxQStatus =
	    ISO9141_14230_AddToQ(ISO9141_14230_TX_Q, (const ISO9141_14230_Q_S *)
				 p_ISO9141_14230TxMsg_SP, 0);

	/* Restore the Tx frame if checksum is configured */
	if (CHECK_BITU8(l_ConnectFlags, BIT_CHKSUM) == FALSE) {

		/* Nullify the changes due to checksum */
		p_ISO9141_14230TxMsg_SP->Length--;
		p_ISO9141_14230TxMsg_SP->Data[p_ISO9141_14230TxMsg_SP->Length] =
		    0;
	}

	/* Update Tx Timestamp */
	//get_time_stamp(timestamp_id[GARUDA_KWP_CH1], &p_ISO9141_14230TxMsg_SP->Timestamp);
	//get_data_logging_time_stamp(&p_ISO9141_14230TxMsg_SP->Timestamp);
	//  get_time_stamp(&p_ISO9141_14230TxMsg_SP->Timestamp);
	/* Check if the queue is Full */
	if (ISO9141_14230_Q_FULL == fl_TxQStatus) {

		/* Send Return Code */
		fl_RetCode = ISO9141_14230_TXQ_FULL;
	}

	/* Return code */
	return fl_RetCode;
}

/******************************************************************************
* Function name     : ISO9141_14230_RETCODE
                              ISO9141_14230_ReadMsg(ISO9141_14230_RxMsg_S
                                                       *p_ISO9141_14230RxMsg_SP)
*    returns        : ISO9141_14230_RETCODE: Returns ISO9141_14230_RXQ_EMPTY
                      if RxQ is empty
*    arg1           : ISO9141_14230_RxMsg_S *p_ISO9141_14230RxMsg_SP
* Description       : This funtion retrieves a frame from the RxQ
* Notes             : restrictions, odd modes
*******************************************************************************/
ISO9141_14230_RETCODE ISO9141_14230_ReadMsg(ISO9141_14230_RxMsg_S
					    * p_ISO9141_14230RxMsg_SP)
{
	ISO9141_14230_RETCODE fl_RetCode = NO_ERROR;
	ISO9141_14230_QSTATUS fl_RxQStatus;

	/* Obtain the message from the Rx queue */
	fl_RxQStatus =
	    ISO9141_14230_DelFromQ(ISO9141_14230_RX_Q, (ISO9141_14230_Q_S *)
				   p_ISO9141_14230RxMsg_SP);

	/* Check if the queue is Empty */
	if (ISO9141_14230_Q_EMPTY == fl_RxQStatus) {

		/* Send Return Code */
		fl_RetCode = ISO9141_14230_RXQ_EMPTY;
	}

	/* Return code */
	return fl_RetCode;
}

/******************************************************************************
*                   P R I V A T E   F U N C T I O N S
*******************************************************************************/

/******************************************************************************
* Function name     : static void ISO9141_14230_LinkInit(void)
*    returns        : void
*    arg1           : void
* Description       : This funtions performs the initialization of communication
                      link parameters for 5 Baud / Fast Init
* Notes             : restrictions, odd modes
*******************************************************************************/
static void ISO9141_14230_LinkInit(void)
{
	/* Start the timer for the initialization */
	//l_InitTimer = start_time_stamp();

	/* Initialize the Value of Bit position for 5 Baud Init */
	l_BitPos = 0;
	if (l_ProtocolId == KWP_PROTOCOL_ID) {
		l_5BaudIdleTime = l_W5;
	}

	else {

		/* Do Nothing */
	}

	/* Determine the Line configuration */
	if (CHECK_BITU8(l_ConnectFlags, BIT_LINECONF) != FALSE) {

		/* K Line only for initialization */
//        Set_Pin_Low(KLINE_LLINE_SLCT);
	}

	else {

		/* Use both K and L line for initialization */
//        Set_Pin_High(KLINE_LLINE_SLCT);
	}

	/* Disable UART till init pattern is generated */
	/* Clear temporary Tx and Rx variable */
	uartClearTxQueue(devicePath, NORMAL_MODE);
	uartClearRxQueue(devicePath, NORMAL_MODE);

	//disable_uart(UART_CH_ID_1);
	// UART_MidDisable(UART_CH_ID_1);
	uartMidDisable(UART_CH_ID_1);

	//UART_ITConfig (UART_CH_ID_1,IER_THRE,DISABLE);
	//UART_ITConfig (UART_CH_ID_1,IER_RBR,DISABLE);

	/* Reset all Buffers */

	/* Reset UART buffer */
	uartClearTxQueue(devicePath, NORMAL_MODE);
	uartClearRxQueue(devicePath, NORMAL_MODE);

	/* Initialize Transmission Queue */
	memset((void *)&l_ISO9141_14230_TxQ, 0,
	       (sizeof(uint8_t) * ISO9141_14230_MAXTXQSIZE));

	/* Initialize Reception Queue */
	memset((void *)&l_ISO9141_14230_RxQ, 0,
	       (sizeof(uint8_t) * ISO9141_14230_MAXRXQSIZE));

	/* Initialize Rx Length Queue */
	memset((void *)&l_ISO9141_14230_RxLenQ, 0,
	       (sizeof(uint8_t) * ISO9141_14230_MAXRXLENQSIZE));

	/* Initialize Rx Time Queue */
	memset((void *)&l_ISO9141_14230_RxTimeQ, 0,
	       (sizeof(uint8_t) * ISO9141_14230_MAXRXTIMEQSIZE));

	/* Clear the RxLength */
	l_RxLength = 0;
	l_MsgReceived = FALSE;
	l_P3MINTimeout = FALSE;
	l_TxComp = FALSE;

	/* Clear the TxLength, Tx index and UART interrupt Tx index */
	l_TxLength = 0;
	l_TxIndex = 0;
	l_UARTIntrTxIndex = 0;

	/* Reset Queue variables */

	/* Rx Queue */
	l_RxQFront = 0;
	l_RxQRear = 0;

	/* Tx Queue */
	l_TxQFront = 0;
	l_TxQRear = 0;

	/* RxLength Queue */
	l_RxLenQFront = 0;
	l_RxLenQRear = 0;

	/* RxLength Queue */
	l_RxTimeQFront = 0;
	l_RxTimeQRear = 0;

	/* Configure UART Tx as GPIO to generate the init pattern */
//    Config_Pin_Output(KW_TXD);
	//  Set_Pin_High(KW_TXD);

	/* Disable RX Time OUT - if some Timeout caused in Idle Detection,
	   The Init will Fail */
	UART_Change_RxTimeOut(UART_CH_ID_1, 0);

	/* Update the reference Idle time - ?? Timer should have started ?? */
	ISO9141_14230_EnableBusMonitor();

	/* Initialization status - Pending */
	l_InitStatus = LINKINIT_PENDING;
}

/******************************************************************************
* Function name     : static void ISO9141_14230_EnableBusMonitor(void)
*    returns        : void
*    arg1           : None
*    arg2           : None
* Description       : Enables the Bus Monitoring for Idle Time by enabling the
                      PIO interrupt on UART Rx pin
* Notes             : restrictions, odd modes
*******************************************************************************/
static void ISO9141_14230_EnableBusMonitor(void)
{
	uint32_t fl_RefIdleTime;

	/* Enable Rising and Falling Edge Int for KW_RX Pin */
	//IO0_INT_EN_R |= 0x00000008;
	//IO0_INT_EN_F |= 0x00000008;

	/* Initialize the reference Idle time */
	//get_time_stamp(l_InitTimer, &fl_RefIdleTime);
	//get_data_logging_time_stamp(&fl_RefIdleTime);
//    get_time_stamp(&fl_RefIdleTime);
	l_RefIdleTime = fl_RefIdleTime;
}

/******************************************************************************
* Function name     : static void ISO9141_14230_DisableBusMonitor(void)
*    returns        : void
*    arg1           : None
*    arg2           : None
* Description       : Disables the Bus Monitoring for Idle Time by disabling the
                      PIO interrupt on UART Rx pin
* Notes             : restrictions, odd modes
*******************************************************************************/
static void ISO9141_14230_DisableBusMonitor(void)
{
	/* Disable Rising and Falling Edge Int for KW_RX Pin */
	//IO0_INT_EN_R &= ~0x00000008;
	//IO0_INT_EN_F &= ~0x00000008;
}

/******************************************************************************
* Function name     : bool ISO9141_14230_GetParity(void)
*    returns        : TRUE or FALSE
*    arg1           : None
*    arg2           : None
* Description       : Checks the Parity of address byte based on Data Bits and
                      the Parity information
* Notes             : restrictions, odd modes
*******************************************************************************/
static bool ISO9141_14230_GetParity(void)
{
	uint8_t fl_Msk = 0;
	uint8_t fl_CntOne = 0;
	bool fl_Ret = FALSE;

	/* Count the number of 1s in the address byte */
	for (fl_Msk = 0; fl_Msk < l_DataBits; fl_Msk++) {

		/* Check if 1 or 0 */
		if (CHECK_BITU8(l_ISO9141_14230LinkInit_S.Data[0], fl_Msk)
		    != FALSE) {
			fl_CntOne++;
		}
	}

	/* If the Count is Odd and if the Parity is Odd parity return TRUE */
	if ((fl_CntOne % 2) != 0) {
		if (l_Parity == ODD_PARITY) {
			fl_Ret = !FALSE;
		}
	}

	/* Count is Even and if the Parity is Even parity return TRUE */
	else if (l_Parity == EVEN_PARITY) {
		fl_Ret = !FALSE;
	}

	else {

		/* Do nothing */
	}
	return fl_Ret;
}

/******************************************************************************
* Function name     : staticuint8_t ISO9141_14230_GetChecksum(
                                           const uint8_t *p_Data,
                                           uint8_t p_Length, uint8_t TxRx)
*    returns        : Checksum Byte or Checksum verification status
*    arg1           : None
*    arg2           : None
* Description       : Calculates the checksum of the data bytes and returns
* Notes             : restrictions, odd modes
*******************************************************************************/
static uint8_t ISO9141_14230_GetChecksum(const uint8_t * p_Data,
					 uint16_t p_Length, uint8_t p_TxRx)
{
	uint8_t fl_Ret = 0;
	uint16_t fl_Cnt = 0;

	/* Calculate the checksum by adding all the bytes till Length - 1 */
	for (fl_Cnt = 0; fl_Cnt < (p_Length - 1); fl_Cnt++) {

		/* Add the bytes */
		fl_Ret = fl_Ret + p_Data[fl_Cnt];
	}

	/* Based on Tx or Rx data determine or verify the checksum respectively */
	if (p_TxRx == CHKSUM_TXDATA) {

		/* For Tx data calculate checksum - Add the last byte as well */
		fl_Ret = fl_Ret + p_Data[p_Length - 1];
	}

	else {

		/* For Rx data verify checksum - Compare with the last byte */
		if (fl_Ret == p_Data[p_Length - 1]) {

			/* For Rx data return 0 if there is no checksum error */
			fl_Ret = CHECKSUM_OK;
		}

		else {

			/* For Rx data return 1 if there is checksum error */
			fl_Ret = CHECKSUM_ERROR;
		}
	}

	/* Return checksum or checksum verification status */
	return fl_Ret;
}

/******************************************************************************
** Function name:		GPIOPinChangeISR
**
** Descriptions:		Ext_Int 3 + Pin Change interrupt handler
**
** parameters:			None
** Returned value:		None
**
******************************************************************************/
void GPIOPinChangeHandler(void)
{
	uint32_t fl_RefIdleTime;

	/* Update the current time. Activity on the bus detected */
	//get_time_stamp(l_InitTimer, &fl_RefIdleTime);
	//get_data_logging_time_stamp(&fl_RefIdleTime);
	//   get_time_stamp(&fl_RefIdleTime);
	l_RefIdleTime = fl_RefIdleTime;
}

/******************************************************************************
* Function name     : static ISO9141_14230_QSTATUS
                      ISO9141_14230_AddToQ(ISO9141_14230_QTYPE p_QType,
                                const ISO9141_14230_Q_S *p_ISO9141_14230Q_SP,
                                uint16_t p_dataoffset)

*    returns        : ISO9141_14230_QSTATUS: Returns Q status(Full or Ok)
*    arg1           : ISO9141_14230_QTYPE p_QType
     arg2             ISO9141_14230_Q_S *p_ISO9141_14230Q_SP
     arg3             uint16_t p_dataoffset

* Description       : This funtion performs enqueue for Tx, Rx and Length Queue
                      based on the Q type
* Notes             : restrictions, odd modes
*******************************************************************************/
static ISO9141_14230_QSTATUS ISO9141_14230_AddToQ(ISO9141_14230_QTYPE p_QType,
						  const ISO9141_14230_Q_S *
						  p_ISO9141_14230Q_SP,
						  uint16_t p_dataoffset)
{
	ISO9141_14230_QSTATUS fl_QStatus = ISO9141_14230_Q_OK;
	uint8_t *ptrByte;
	uint16_t numBytes = 0, i = 0, rear = 0, front = 0, max = 0;

	/* Based on Q type determine num of bytes to store, rear and front pointers
	   and max elements */
	if (ISO9141_14230_RX_Q == p_QType) {

		/* Number of bytes to store is determined by the size of individual
		   members of structure and the length of the data bytes */
		numBytes =
		    sizeof(p_ISO9141_14230Q_SP->p_ISO9141_14230RxMsg_S.
			   Timestamp) +
		    sizeof(p_ISO9141_14230Q_SP->p_ISO9141_14230RxMsg_S.Length) +
		    sizeof(p_ISO9141_14230Q_SP->p_ISO9141_14230RxMsg_S.Flags) +
		    p_ISO9141_14230Q_SP->p_ISO9141_14230RxMsg_S.Length;
		rear = l_RxQRear;
		front = l_RxQFront;
		max = ISO9141_14230_MAXRXQSIZE;
	}

	else if (ISO9141_14230_TX_Q == p_QType) {

		/* Number of bytes to store is determined by the size of individual
		   members of structure and the length of the data bytes */
		numBytes =
		    sizeof(p_ISO9141_14230Q_SP->p_ISO9141_14230TxMsg_S.Length) +
		    sizeof(p_ISO9141_14230Q_SP->p_ISO9141_14230TxMsg_S.Flags) +
		    p_ISO9141_14230Q_SP->p_ISO9141_14230TxMsg_S.Length;
		rear = l_TxQRear;
		front = l_TxQFront;
		max = ISO9141_14230_MAXTXQSIZE;
	}

	else if (ISO9141_14230_LEN_Q == p_QType) {

		/* Number of bytes to store is determined by the size of RxLength */
		numBytes = sizeof(p_ISO9141_14230Q_SP->RxLength);
		rear = l_RxLenQRear;
		front = l_RxLenQFront;
		max = ISO9141_14230_MAXRXLENQSIZE;
	}

	else {

		/* Number of bytes to store is determined by the size of RxTime */
		numBytes = sizeof(p_ISO9141_14230Q_SP->RxTime);
		rear = l_RxTimeQRear;
		front = l_RxTimeQFront;
		max = ISO9141_14230_MAXRXTIMEQSIZE;
	}

	/* Check if queue is full */
	if (((rear >= front) && ((max - rear + front) < numBytes)) ||
	    ((rear < front) && ((front - rear) < numBytes))) {

		/* Return Queue Full */
		fl_QStatus = ISO9141_14230_Q_FULL;
	}

	/* If Q not full store the elements */
	else {

		/* Rx Q */
		if (ISO9141_14230_RX_Q == p_QType) {

			/* Save Timestamp bytes */
			i = 0;

			/* Number of bytes is the size of the Timestamp */
			numBytes =
			    sizeof(p_ISO9141_14230Q_SP->p_ISO9141_14230RxMsg_S.
				   Timestamp);

			/* Store the address of the Timestamp onto the byte pointer */
			ptrByte = (uint8_t *)
			    & p_ISO9141_14230Q_SP->p_ISO9141_14230RxMsg_S.
			    Timestamp;

			/* Copy the bytes */
			while (i < numBytes) {

				/* If the index reaches max then start from 0 */
				if (l_RxQRear == max) {
					l_RxQRear = 0;
				}

				/* Store the byte */
				l_ISO9141_14230_RxQ[l_RxQRear++] =
				    *(ptrByte + i);
				i++;
			}

			/* Save Msg Length bytes */
			i = 0;

			/* Number of bytes is the size of the Timestamp */
			numBytes =
			    sizeof(p_ISO9141_14230Q_SP->p_ISO9141_14230RxMsg_S.
				   Length);

			/* Store the address of the Length onto the byte pointer */
			ptrByte = (uint8_t *)
			    & p_ISO9141_14230Q_SP->
			    p_ISO9141_14230RxMsg_S.Length;

			/* Copy the bytes */
			while (i < numBytes) {

				/* If the index reaches max then start from 0 */
				if (l_RxQRear == max) {
					l_RxQRear = 0;
				}

				/* Store the byte */
				l_ISO9141_14230_RxQ[l_RxQRear++] =
				    *(ptrByte + i);
				i++;
			}

			/* Save the data bytes */
			i = 0;

			/* Copy the bytes */
			while (i <
			       p_ISO9141_14230Q_SP->p_ISO9141_14230RxMsg_S.
			       Length) {

				/* If the index reaches max then start from 0 */
				if (l_RxQRear == max) {
					l_RxQRear = 0;
				}

				/* Store the byte */
				l_ISO9141_14230_RxQ[l_RxQRear++] =
				    p_ISO9141_14230Q_SP->
				    p_ISO9141_14230RxMsg_S.Data[i +
								p_dataoffset];
				i++;
			}

			/* Save Flag bytes */
			i = 0;

			/* Number of bytes is the size of the Flags */
			numBytes =
			    sizeof(p_ISO9141_14230Q_SP->p_ISO9141_14230RxMsg_S.
				   Flags);

			/* Store the address of the Flags onto the byte pointer */
			ptrByte = (uint8_t *)
			    & p_ISO9141_14230Q_SP->p_ISO9141_14230RxMsg_S.Flags;

			/* Copy the bytes */
			while (i < numBytes) {

				/* If the index reaches max then start from 0 */
				if (l_RxQRear == max) {
					l_RxQRear = 0;
				}

				/* Store the byte */
				l_ISO9141_14230_RxQ[l_RxQRear++] =
				    *(ptrByte + i);
				i++;
			}
		}

		/* Tx Q */
		else if (ISO9141_14230_TX_Q == p_QType) {

			/* Save Msg Length bytes */
			i = 0;

			/* Number of bytes is the size of the Length */
			numBytes =
			    sizeof(p_ISO9141_14230Q_SP->p_ISO9141_14230TxMsg_S.
				   Length);

			/* Store the address of the Length onto the byte pointer */
			ptrByte = (uint8_t *)
			    & p_ISO9141_14230Q_SP->
			    p_ISO9141_14230TxMsg_S.Length;

			/* Copy the bytes */
			while (i < numBytes) {

				/* If the index reaches max then start from 0 */
				if (l_TxQRear == max) {
					l_TxQRear = 0;
				}

				/* Store the byte */
				l_ISO9141_14230_TxQ[l_TxQRear++] =
				    *(ptrByte + i);
				i++;
			}

			/* Save the data bytes */
			i = 0;

			/* Copy the bytes */
			while (i <
			       p_ISO9141_14230Q_SP->p_ISO9141_14230TxMsg_S.
			       Length) {

				/* If the index reaches max then start from 0 */
				if (l_TxQRear == max) {
					l_TxQRear = 0;
				}

				/* Store the byte */
				l_ISO9141_14230_TxQ[l_TxQRear++] =
				    p_ISO9141_14230Q_SP->
				    p_ISO9141_14230TxMsg_S.Data[i +
								p_dataoffset];
				i++;
			}

			/* Save Flag bytes */
			i = 0;

			/* Number of bytes is the size of the Flags */
			numBytes =
			    sizeof(p_ISO9141_14230Q_SP->p_ISO9141_14230TxMsg_S.
				   Flags);

			/* Store the address of the Flags onto the byte pointer */
			ptrByte = (uint8_t *)
			    & p_ISO9141_14230Q_SP->p_ISO9141_14230TxMsg_S.Flags;

			/* Copy the bytes */
			while (i < numBytes) {

				/* If the index reaches max then start from 0 */
				if (l_TxQRear == max) {
					l_TxQRear = 0;
				}

				/* Store the byte */
				l_ISO9141_14230_TxQ[l_TxQRear++] =
				    *(ptrByte + i);
				i++;
			}
		}

		/* RxLength Q */
		else if (ISO9141_14230_LEN_Q == p_QType) {

			/* Save RxLength bytes */
			i = 0;

			/* Number of bytes is the size of the RxLength */
			numBytes = sizeof(p_ISO9141_14230Q_SP->RxLength);

			/* Store the address of the RxLength onto the byte pointer */
			ptrByte = (uint8_t *) & p_ISO9141_14230Q_SP->RxLength;

			/* Copy the bytes */
			while (i < numBytes) {

				/* If the index reaches max then start from 0 */
				if (l_RxLenQRear == max) {
					l_RxLenQRear = 0;
				}

				/* Store the byte */
				l_ISO9141_14230_RxLenQ[l_RxLenQRear++] =
				    *(ptrByte + i);
				i++;
			}
		}

		/* RxTime Q */
		else {

			/* Save RxTime bytes */
			i = 0;

			/* Number of bytes is the size of the RxTime */
			numBytes = sizeof(p_ISO9141_14230Q_SP->RxTime);

			/* Store the address of the RxTime onto the byte pointer */
			ptrByte = (uint8_t *) & p_ISO9141_14230Q_SP->RxTime;

			/* Copy the bytes */
			while (i < numBytes) {

				/* If the index reaches max then start from 0 */
				if (l_RxTimeQRear == max) {
					l_RxTimeQRear = 0;
				}

				/* Store the byte */
				l_ISO9141_14230_RxTimeQ[l_RxTimeQRear++] =
				    *(ptrByte + i);
				i++;
			}
		}
	}
	return fl_QStatus;;
}

/******************************************************************************
* Function name     : static ISO9141_14230_QSTATUS
                      ISO9141_14230_DelFromQ(ISO9141_14230_QTYPE p_QType,
                                  ISO9141_14230_Q_S *p_ISO9141_14230Q_SP)

*    returns        : ISO9141_14230_QSTATUS: Returns Q status(Empty or Ok)
*    arg1           : ISO9141_14230_QTYPE p_QType
     arg2             ISO9141_14230_Q_S *p_ISO9141_14230Q_SP

* Description       : This funtion performs dequeue for Tx, Rx and Length Queue
                      based on the Q type
* Notes             : restrictions, odd modes
*******************************************************************************/
static ISO9141_14230_QSTATUS ISO9141_14230_DelFromQ(ISO9141_14230_QTYPE
						    p_QType,
						    ISO9141_14230_Q_S *
						    p_ISO9141_14230Q_SP)
{
	ISO9141_14230_QSTATUS fl_QStatus = ISO9141_14230_Q_OK;
	uint8_t *ptrByte;
	uint16_t numBytes = 0, i = 0, rear = 0, front = 0, max = 0;

	/* Based on Q type determine rear, front pointers and max elements */
	if (ISO9141_14230_RX_Q == p_QType) {
		rear = l_RxQRear;
		front = l_RxQFront;
		max = ISO9141_14230_MAXRXQSIZE;
	}

	else if (ISO9141_14230_TX_Q == p_QType) {
		rear = l_TxQRear;
		front = l_TxQFront;
		max = ISO9141_14230_MAXTXQSIZE;
	}

	else if (ISO9141_14230_LEN_Q == p_QType) {
		rear = l_RxLenQRear;
		front = l_RxLenQFront;
		max = ISO9141_14230_MAXRXLENQSIZE;
	}

	else {
		rear = l_RxTimeQRear;
		front = l_RxTimeQFront;
		max = ISO9141_14230_MAXRXTIMEQSIZE;
	}

	/* Check if queue is empty */
	if (rear == front) {
		fl_QStatus = ISO9141_14230_Q_EMPTY;
	}

	/* If Q not Empty retrieve the elements */
	else {

		/* Rx Q */
		if (ISO9141_14230_RX_Q == p_QType) {

			/* Retrieve Timestamp bytes */
			i = 0;

			/* Number of bytes is the size of the Timestamp */
			numBytes =
			    sizeof(p_ISO9141_14230Q_SP->p_ISO9141_14230RxMsg_S.
				   Timestamp);

			/* Store the address of the Timestamp onto the byte pointer */
			ptrByte = (uint8_t *)
			    & p_ISO9141_14230Q_SP->p_ISO9141_14230RxMsg_S.
			    Timestamp;

			/* Copy the bytes */
			while (i < numBytes) {

				/* If the index reaches max then start from 0 */
				if (l_RxQFront == max) {
					l_RxQFront = 0;
				}

				/* Retrieve the byte */
				*(ptrByte + i) =
				    l_ISO9141_14230_RxQ[l_RxQFront++];
				i++;
			}

			/* Save Msg Length bytes */
			i = 0;

			/* Number of bytes is the size of the Length */
			numBytes =
			    sizeof(p_ISO9141_14230Q_SP->p_ISO9141_14230RxMsg_S.
				   Length);

			/* Store the address of the Length onto the byte pointer */
			ptrByte = (uint8_t *)
			    & p_ISO9141_14230Q_SP->
			    p_ISO9141_14230RxMsg_S.Length;

			/* Copy the bytes */
			while (i < numBytes) {

				/* If the index reaches max then start from 0 */
				if (l_RxQFront == max) {
					l_RxQFront = 0;
				}

				/* Retrieve the byte */
				*(ptrByte + i) =
				    l_ISO9141_14230_RxQ[l_RxQFront++];
				i++;
			}

			/* Get the data bytes */
			i = 0;
			while (i <
			       p_ISO9141_14230Q_SP->p_ISO9141_14230RxMsg_S.
			       Length) {

				/* If the index reaches max then start from 0 */
				if (l_RxQFront == max) {
					l_RxQFront = 0;
				}

				/* Retrieve the byte */
				p_ISO9141_14230Q_SP->p_ISO9141_14230RxMsg_S.
				    Data[i] = l_ISO9141_14230_RxQ[l_RxQFront++];
				i++;
			}

			/* Save Flag bytes */
			i = 0;

			/* Number of bytes is the size of the Flags */
			numBytes =
			    sizeof(p_ISO9141_14230Q_SP->p_ISO9141_14230RxMsg_S.
				   Flags);

			/* Store the address of the Flags onto the byte pointer */
			ptrByte = (uint8_t *)
			    & p_ISO9141_14230Q_SP->p_ISO9141_14230RxMsg_S.Flags;

			/* Get the bytes */
			while (i < numBytes) {

				/* If the index reaches max then start from 0 */
				if (l_RxQFront == max) {
					l_RxQFront = 0;
				}

				/* Retrieve the byte */
				*(ptrByte + i) =
				    l_ISO9141_14230_RxQ[l_RxQFront++];
				i++;
			}
		}

		/* Tx Q */
		else if (ISO9141_14230_TX_Q == p_QType) {

			/* Save Msg Length bytes */
			i = 0;

			/* Number of bytes is the size of the Length */
			numBytes =
			    sizeof(p_ISO9141_14230Q_SP->p_ISO9141_14230TxMsg_S.
				   Length);

			/* Store the address of the Length onto the byte pointer */
			ptrByte = (uint8_t *)
			    & p_ISO9141_14230Q_SP->
			    p_ISO9141_14230TxMsg_S.Length;

			/* Get the bytes */
			while (i < numBytes) {

				/* If the index reaches max then start from 0 */
				if (l_TxQFront == max) {
					l_TxQFront = 0;
				}

				/* Retrieve the byte */
				*(ptrByte + i) =
				    l_ISO9141_14230_TxQ[l_TxQFront++];
				i++;
			}

			/* Retrieve the data bytes */
			i = 0;
			while (i <
			       p_ISO9141_14230Q_SP->p_ISO9141_14230TxMsg_S.
			       Length) {

				/* If the index reaches max then start from 0 */
				if (l_TxQFront == max) {
					l_TxQFront = 0;
				}

				/* Retrieve the byte */
				p_ISO9141_14230Q_SP->p_ISO9141_14230TxMsg_S.
				    Data[i] = l_ISO9141_14230_TxQ[l_TxQFront++];
				i++;
			}

			/* Retrieve Flag bytes */
			i = 0;

			/* Number of bytes is the size of the Flags */
			numBytes =
			    sizeof(p_ISO9141_14230Q_SP->p_ISO9141_14230TxMsg_S.
				   Flags);

			/* Store the address of the Flags onto the byte pointer */
			ptrByte = (uint8_t *)
			    & p_ISO9141_14230Q_SP->p_ISO9141_14230TxMsg_S.Flags;

			/* Retrieve the bytes */
			while (i < numBytes) {

				/* If the index reaches max then start from 0 */
				if (l_TxQFront == max) {
					l_TxQFront = 0;
				}

				/* Retrieve the byte */
				*(ptrByte + i) =
				    l_ISO9141_14230_TxQ[l_TxQFront++];
				i++;
			}
		}

		/* RxLength Q */
		else if (ISO9141_14230_LEN_Q == p_QType) {

			/* Save RxLength bytes */
			i = 0;

			/* Number of bytes is the size of the RxLength */
			numBytes = sizeof(p_ISO9141_14230Q_SP->RxLength);

			/* Store the address of the RxLength onto the byte pointer */
			ptrByte = (uint8_t *) & p_ISO9141_14230Q_SP->RxLength;

			/* Retrieve the bytes */
			while (i < numBytes) {

				/* If the index reaches max then start from 0 */
				if (l_RxLenQFront == max) {
					l_RxLenQFront = 0;
				}

				/* Retrieve the byte */
				*(ptrByte + i) =
				    l_ISO9141_14230_RxLenQ[l_RxLenQFront++];
				i++;
			}
		}

		/* RxTime Q */
		else {

			/* Save RxTime bytes */
			i = 0;

			/* Number of bytes is the size of the RxTime */
			numBytes = sizeof(p_ISO9141_14230Q_SP->RxTime);

			/* Store the address of the RxTime onto the byte pointer */
			ptrByte = (uint8_t *) & p_ISO9141_14230Q_SP->RxTime;

			/* Retrieve the bytes */
			while (i < numBytes) {

				/* If the index reaches max then start from 0 */
				if (l_RxTimeQFront == max) {
					l_RxTimeQFront = 0;
				}

				/* Retrieve the byte */
				*(ptrByte + i) =
				    l_ISO9141_14230_RxTimeQ[l_RxTimeQFront++];
				i++;
			}
		}
	}
	return fl_QStatus;;
}

/******************************************************************************
* Function name     : static ISO9141_14230_QSTATUS
                      ISO9141_14230_ReadFromQ(ISO9141_14230_QTYPE p_QType,
                                  ISO9141_14230_Q_S *p_ISO9141_14230Q_SP)

*    returns        : ISO9141_14230_QSTATUS: Returns Q status(Empty or Ok)
*    arg1           : ISO9141_14230_QTYPE p_QType
     arg2             ISO9141_14230_Q_S *p_ISO9141_14230Q_SP

* Description       : This funtion performs a read for Tx, Rx and Length Queue
                      based on the Q type. Contents of the Q are not deleted.
* Notes             : restrictions, odd modes
*******************************************************************************/
static ISO9141_14230_QSTATUS ISO9141_14230_ReadFromQ(ISO9141_14230_QTYPE
						     p_QType,
						     ISO9141_14230_Q_S *
						     p_ISO9141_14230Q_SP)
{
	ISO9141_14230_QSTATUS fl_QStatus = ISO9141_14230_Q_OK;
	uint8_t *ptrByte;
	uint16_t numBytes = 0, i = 0, rear = 0, front = 0, max = 0;

	/* Based on Q type determine rear, front pointers and max elements */
	if (ISO9141_14230_RX_Q == p_QType) {
		rear = l_RxQRear;
		front = l_RxQFront;
		max = ISO9141_14230_MAXRXQSIZE;
	}

	else if (ISO9141_14230_TX_Q == p_QType) {
		rear = l_TxQRear;
		front = l_TxQFront;
		max = ISO9141_14230_MAXTXQSIZE;
	}

	else if (ISO9141_14230_LEN_Q == p_QType) {
		rear = l_RxLenQRear;
		front = l_RxLenQFront;
		max = ISO9141_14230_MAXRXLENQSIZE;
	}

	else {
		rear = l_RxTimeQRear;
		front = l_RxTimeQFront;
		max = ISO9141_14230_MAXRXTIMEQSIZE;
	}

	/* Check if queue is empty */
	if (rear == front) {
		fl_QStatus = ISO9141_14230_Q_EMPTY;
	}

	/* If Q not Empty retrieve the elements */
	else {

		/* Rx Q */
		if (ISO9141_14230_RX_Q == p_QType) {

			/* Retrieve Timestamp bytes */
			i = 0;
			numBytes =
			    sizeof(p_ISO9141_14230Q_SP->p_ISO9141_14230RxMsg_S.
				   Timestamp);
			ptrByte =
			    (uint8_t *) & p_ISO9141_14230Q_SP->
			    p_ISO9141_14230RxMsg_S.Timestamp;
			while (i < numBytes) {
				if (front == max) {
					front = 0;
				}
				*(ptrByte + i) = l_ISO9141_14230_RxQ[front++];
				i++;
			}

			/* Save Msg Length bytes */
			i = 0;
			numBytes =
			    sizeof(p_ISO9141_14230Q_SP->p_ISO9141_14230RxMsg_S.
				   Length);
			ptrByte =
			    (uint8_t *) & p_ISO9141_14230Q_SP->
			    p_ISO9141_14230RxMsg_S.Length;
			while (i < numBytes) {
				if (front == max) {
					front = 0;
				}
				*(ptrByte + i) = l_ISO9141_14230_RxQ[front++];
				i++;
			}

			/* Save the data bytes */
			i = 0;
			while (i <
			       p_ISO9141_14230Q_SP->p_ISO9141_14230RxMsg_S.
			       Length) {
				if (front == max) {
					front = 0;
				}
				p_ISO9141_14230Q_SP->p_ISO9141_14230RxMsg_S.
				    Data[i] = l_ISO9141_14230_RxQ[front++];
				i++;
			}

			/* Save Flag bytes */
			i = 0;
			numBytes =
			    sizeof(p_ISO9141_14230Q_SP->p_ISO9141_14230RxMsg_S.
				   Flags);
			ptrByte =
			    (uint8_t *) & p_ISO9141_14230Q_SP->
			    p_ISO9141_14230RxMsg_S.Flags;
			while (i < numBytes) {
				if (front == max) {
					front = 0;
				}
				*(ptrByte + i) = l_ISO9141_14230_RxQ[front++];
				i++;
			}
		}

		/* Tx Q */
		else if (ISO9141_14230_TX_Q == p_QType) {

			/* Save Msg Length bytes */
			i = 0;
			numBytes =
			    sizeof(p_ISO9141_14230Q_SP->p_ISO9141_14230TxMsg_S.
				   Length);
			ptrByte =
			    (uint8_t *) & p_ISO9141_14230Q_SP->
			    p_ISO9141_14230TxMsg_S.Length;
			while (i < numBytes) {
				if (front == max) {
					front = 0;
				}
				*(ptrByte + i) = l_ISO9141_14230_TxQ[front++];
				i++;
			}

			/* Save the data bytes */
			i = 0;
			while (i <
			       p_ISO9141_14230Q_SP->p_ISO9141_14230TxMsg_S.
			       Length) {
				if (front == max) {
					front = 0;
				}
				p_ISO9141_14230Q_SP->p_ISO9141_14230TxMsg_S.
				    Data[i] = l_ISO9141_14230_TxQ[front++];
				i++;
			}

			/* Save Flag bytes */
			i = 0;
			numBytes =
			    sizeof(p_ISO9141_14230Q_SP->p_ISO9141_14230TxMsg_S.
				   Flags);
			ptrByte =
			    (uint8_t *) & p_ISO9141_14230Q_SP->
			    p_ISO9141_14230TxMsg_S.Flags;
			while (i < numBytes) {
				if (front == max) {
					front = 0;
				}
				*(ptrByte + i) = l_ISO9141_14230_TxQ[front++];
				i++;
			}
		}

		/* RxLength Q */
		else if (ISO9141_14230_LEN_Q == p_QType) {

			/* Save RxLength bytes */
			i = 0;
			numBytes = sizeof(p_ISO9141_14230Q_SP->RxLength);
			ptrByte = (uint8_t *) & p_ISO9141_14230Q_SP->RxLength;
			while (i < numBytes) {
				if (front == max) {
					front = 0;
				}
				*(ptrByte + i) =
				    l_ISO9141_14230_RxLenQ[front++];
				i++;
			}
		}

		/* RxTime Q */
		else {

			/* Save RxTime bytes */
			i = 0;
			numBytes = sizeof(p_ISO9141_14230Q_SP->RxTime);
			ptrByte = (uint8_t *) & p_ISO9141_14230Q_SP->RxTime;
			while (i < numBytes) {
				if (front == max) {
					front = 0;
				}
				*(ptrByte + i) =
				    l_ISO9141_14230_RxTimeQ[front++];
				i++;
			}
		}
	}
	return fl_QStatus;;
}

/******************************************************************************
* Function name     : static ISO9141_14230_QSTATUS
                      ISO9141_14230_GetByteFromQ(ISO9141_14230_QTYPE p_QType,
                                  uint16_t p_Index, uint8_t *p_RetDataByte)

*    returns        : ISO9141_14230_QSTATUS: Returns Q status(Empty or Ok)
*    arg1           : ISO9141_14230_QTYPE p_QType
     arg2           : uint16_t p_Index
     arg3           : uint8_t *p_RetDataByte

* Description       : This funtion retrieves a byte from the data member
                      of Tx or Rx Queue based on the Q type.
* Notes             : restrictions, odd modes
*******************************************************************************/
static ISO9141_14230_QSTATUS ISO9141_14230_GetByteFromQ(ISO9141_14230_QTYPE
							p_QType,
							uint16_t p_Index,
							uint8_t * p_RetDataByte)
{
	ISO9141_14230_QSTATUS fl_QStatus = ISO9141_14230_Q_OK;
	ISO9141_14230_Q_S fl_ISO9141_14230Q_S;
	uint16_t rear = 0, front = 0, max = 0;

	/* Based on Q type determine rear, front pointers and max elements */
	if (ISO9141_14230_TX_Q == p_QType) {
		rear = l_TxQRear;
		front = l_TxQFront;
		max = ISO9141_14230_MAXTXQSIZE;
	}

	else if (ISO9141_14230_RX_Q == p_QType) {
		rear = l_RxQRear;
		front = l_RxQFront;
		max = ISO9141_14230_MAXRXQSIZE;
	}

	else {

		/* Wrong Q Id - Data byte not possible for RxLength and RxTime Queue */
		/* Do nothing */
	}

	/* Check if queue is empty */
	if (rear == front) {
		fl_QStatus = ISO9141_14230_Q_EMPTY;
	}

	/* If Q not Empty retrieve the elements */
	else {

		/* Tx Q */
		if (ISO9141_14230_TX_Q == p_QType) {

			/* Retrieve the data byte from the Index */

			/* Determine the index from where data byte is to be obtained */
			front =
			    front +
			    sizeof(fl_ISO9141_14230Q_S.p_ISO9141_14230TxMsg_S.
				   Length) + p_Index;

			/* If front exceeds maximum then reset the index */
			if (front >= max) {
				front = front - max;
			}
			*p_RetDataByte = l_ISO9141_14230_TxQ[front];
		}

		/* Rx Q */
		else if (ISO9141_14230_RX_Q == p_QType) {

			/* Retrieve the data byte from the Index */

			/* Determine the index from where data byte is to be obtained */
			front =
			    front +
			    sizeof(fl_ISO9141_14230Q_S.p_ISO9141_14230RxMsg_S.
				   Timestamp) +
			    sizeof(fl_ISO9141_14230Q_S.p_ISO9141_14230RxMsg_S.
				   Length) + p_Index;

			/* If front exceeds maximum then reset the index */
			if (front >= max) {
				front = front - max;
			}
			*p_RetDataByte = l_ISO9141_14230_RxQ[front];
		}

		else {

			/* Do nothing */
		}
	}
	return fl_QStatus;;
}

/*******************************************************************************
* Function Name  : clearP1MaxExtendedTimeoutValue
* Description    : Validates the 14230 frames,  based on the length
* Input          : void
* Output         : None
* Return         : void
* Author         : Amit: Tata Motors Fix for delayed interbyte time
*******************************************************************************/
void clearP1MaxExtendedTimeoutValue(void)
{
	l_P1MAX_ExtendedTimeout = 0;
}

/*
static void vKwpTimerCallback( TimerHandle_t pxTimer ) 
{
  UART_timeguard_handler();
  UART_RXtimeout_handler();
  ISO9141_14230_TimerIntrHandler();
}
*/

/******************************************************************************
 * R E V I S I O N   H I S T O R Y
 * $History: $
 * Version  Author  Date
 * 1.0  Karthik Subramanian    April 10, 2008
 * 2.0  Karthik Subramanian    April 28, 2008
 * 3.0  Karthik Subramanian    May   15, 2008
********************************************************************************
 * 1.0  Initial Version
 * 2.0  Changes to the Queue handling to account for 4128 bytes. Addition of
        new queue functions ReadFromQ and GetByteFromQueue. Changes in UART
        interrupt handler and Tx task due to change in queue handling. Separate
        variables to handle Init time and Tx time.
 * 3.0  Changes to the P3MIN timeout handling.
 * 4.0  Changes to Initialization in TimerIntrHandler and UARTIntrHandler.
        W4 minimum time wait for Transmitting Keybyte 2 inverse. W4 max timeout
        wait. Separate timer for initialization pattern, changes in timestamp
        functions and P3 min wait after Init.
*******************************************************************************/

/* 5.0      Mahadev Jun 24, 2009
 * Modified the For LPC 2468 (GARUDA)
 * Time Out and Timeguard Are Implemented Using the Timers
 * Bus Idle Moniter ( Enable, Disable Bus Monitor and ISRs) Changed
 * Queue Sizes Modified
 * Time OUt Disabled Before Initialization Starts
 * Setconfig l_p4min made '0' to Increase Flash Speed
*******************************************************************************/

/* 5.1   Mahadev  Jul 07,2009
 * Updated Changes for UART1 use for KWP Drivers
*******************************************************************************/

/* 6.0   Sanjeeva & Mahadeva  Jan 20,2014
 * Wifi Revision *
 * UART 1 is replaced by UART 0, to utilize the modem uart for Wifi Commuication
 * Timestamps for Initialization changed to new timestamp implemetation.
*******************************************************************************/

/*******************************************************************************
* 6.1   Sanjeeva & Mahadeva  Mar 06,2014
* uart.h Removed
*******************************************************************************/

/*******************************************************************************
* 6.2   Mahadeva  Apr 21,2014
* Get_KLPullUp renamed as CheckIf_VBATTis24V. and this function return proper
battery Vg instead earlier hardcode value
*******************************************************************************/

/*******************************************************************************
* 6.3   Mahadeva  Oct 10,2014
* Firmware changed for new schematics V1.8 
1)K & L lines isolation from the J1962.
2)Changes for Pullup seletion for K & L line
*******************************************************************************/
