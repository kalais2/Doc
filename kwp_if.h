#ifndef _KWP_IF_H_
#define	_KWP_IF_H_

#include	<glib.h>

#include	"g3d.h"

#include        <stdbool.h>
/* macro */
/* IOCTL Command IDs */
#define	GET_CONFIG                        		            (0x01)
#define	SET_CONFIG                        		            (0x02)
#define	READ_VBATT                        		            (0x03)
#define	FIVE_BAUD_INIT                    		            (0x04)
#define	FAST_INIT                         		            (0x05)
#define	CLEAR_TX_BUFFER                   		            (0x07)
#define	CLEAR_RX_BUFFER                   		            (0x08)

/* GET_CONFIG and SET_CONFIG Ids */

/* General */
#define DATA_RATE    (0x01)
#define	LOOPBACK     (0x03)

/* KWP Ids */
#define P1_MAX        0x07
#define P3_MIN        0x0A
#define P4_MIN        0x0C
#define W0            0x19
#define W1            0x0E
#define W2            0x0F
#define W3            0x10
#define W4            0x11
#define W5            0x12
#define TIDLE         0x13
#define TINIL         0x14
#define TWUP          0x15
#define PARITY        0x16
#define DATA_BITS     0x20
#define FIVE_BAUD_MOD 0x21

/* Maximum length for Tx and Rx frame */
#define ISO9141_14230_MAXRXMSGLEN 260
#define ISO9141_14230_MAXTXMSGLEN 260

/* Queue size for Rx, Tx and Length */
#define ISO9141_14230_MAXRXQSIZE     (2700)	/* 259 bytes data+ 2 byte len + 4 byte flag + 4 byte timestamp */
#define ISO9141_14230_MAXTXQSIZE     (2700)
#define ISO9141_14230_MAXRXLENQSIZE  (20)
#define ISO9141_14230_MAXRXTIMEQSIZE (2*ISO9141_14230_MAXRXLENQSIZE)

/* Check if a particular bit is set or reset */
#define CHECK_BITU8(X,n) (X & ((uint8_t)0x1 << (n)))
#define CHECK_BITU32(X,n) (X & ((uint32_t)0x1 << (n)))
#define SET_BITU8(X,n) (X |= ((uint8_t)0x1 << (n)))

/* Length of initialization data */
#define ISO9141_14230_MAXINITDATA 25

/* Five baud time period in us */
#define FIVE_BAUD_TIME  ((uint32_t)200000)
#define W4_MAX_TIME     ((uint32_t)50000)
#define P2_MAX_TIME     ((uint32_t)500000)
#define P1_MAX_EXTENDED_TIMEOUT ((uint32_t)500000)


/* Status of initialization of the communication link */
#define NO_LINKINIT      ((uint8_t)0)
#define LINKINIT_PENDING ((uint8_t)1)
#define LINKINIT_DONE    ((uint8_t)2)
#define LINKINIT_FAIL    ((uint8_t)3)

/* Bit position for transmission of the 5 baud address byte */
#define START_BIT_POS ((uint8_t)0)
#define STOP_BIT_POS  ((uint8_t)10)

/* Number of bits for the data */
#define EIGHT_BITS  ((uint8_t)8)
#define SEVEN_BITS  ((uint8_t)7)

/* Parity configuration */
#define NO_PARITY   ((uint8_t)0)
#define ODD_PARITY  ((uint8_t)1)
#define EVEN_PARITY ((uint8_t)2)

/* Bit coded positions for Init flags */

/* K & L line configuration bit position */
#define BIT_LINECONF ((uint8_t)0)

/* Checksum configuration bit position */
#define BIT_CHKSUM   ((uint8_t)1)

/* Type of data for the checksum determination */
#define CHKSUM_TXDATA ((uint8_t)0)
#define CHKSUM_RXDATA ((uint8_t)1)

/* Five baud mode configuration */
#define FIVEBAUD_MODE_ZERO   ((uint8_t)0x00)
#define FIVEBAUD_MODE_ONE    ((uint8_t)0x01)
#define FIVEBAUD_MODE_TWO    ((uint8_t)0x02)
#define FIVEBAUD_MODE_THREE  ((uint8_t)0x03)

/* Bit position for 5 Baud Init bytes */
#define PATTERN_BYTE_POS     ((uint8_t)0x00)
#define KEY_BYTE1_POS        ((uint8_t)0x01)
#define KEY_BYTE2_POS        ((uint8_t)0x02)
#define ADDRESS_BYTE_INV_POS ((uint8_t)0x03)

/* Bit position for Fast Init wakeup pattern */
#define WKUP_TINIL      ((uint8_t)0)
#define WKUP_TWUP       ((uint8_t)1)
#define WKUP_STARTREQ   ((uint8_t)2)

/* Checksum configuration */
#define CALC_CHECKSUM    ((uint8_t)0)
#define NO_CHECKSUM      ((uint8_t)1)

/* Checksum error status */
#define CHECKSUM_OK      ((uint8_t)0)
#define CHECKSUM_ERROR   ((uint8_t)1)

/* Error Codes for Application Handler */
#define FASTINIT_RESP_TIMEOUT         ((uint8_t)0x01)
#define FASTINIT_RESPCHKSUM_ERROR     ((uint8_t)0x02)
#define PATTERN_BYTE_TIMEOUT          ((uint8_t)0x03)
#define KEY_BYTE1_TIMEOUT             ((uint8_t)0x04)
#define KEY_BYTE2_TIMEOUT             ((uint8_t)0x05)
#define ADDRESS_BYTE_TIMEOUT          ((uint8_t)0x06)
#define MSG_CORRUPTED                 ((uint8_t)0x07)
#define ECU_RESP_TIMEOUT              ((uint8_t)0x08)
#define ECU_RESPCHKSUM_ERROR          ((uint8_t)0x09)
#define RXQ_OVERFLOW                  ((uint8_t)0x0A)

/* UART Interrupt types */
#define RX_INTR                      ((uint8_t)0x00)
#define TX_INTR                      ((uint8_t)0x01)
#define RXTIMEOUT_INTR               ((uint8_t)0x02)

/* RxFlags for received frame */
#define RX_MSG_RECEIVED              0x02

/* Bit positions of connect flags */
#define ISO9141_K_LINE_ONLY_BIT_POS   ((uint32_t)12)
#define ISO9141_NO_CHECKSUM_BIT_POS   ((uint32_t)9)

#define FRAME_FORMAT_MODE_BITS          0xC0
#define FRAME_FORMAT_HDR_LENGTH_BITS    0x3F
#define FRAME_FORMAT_HDR_LEN_ZERO       0x00

#define HDR_FRM_CK                      2
#define HDR_FRM_TA_SA_CK                4
#define HDR_FRM_LEN_CK                  3
#define HDR_FRM_TA_SA_LEN_CK            5

#define LENGTH_INDEX_WITH_ADDR          3
#define LENGTH_INDEX_WO_ADDR            1

#define ADDRESS_PRESENT                 1
#define NO_ADDRESS                      0


/* KWP_Time_handler */
#define TIME_COUNTER_TO_MS (timer_counter*1000)


typedef struct {
        uint32_t Baudrate;      /* Baudrate for UART */
        uint32_t Flags;         /* Bit coded K & L and checksum
                                   configuration :
                                   0th Bit specifies K & L line
                                   config and 1st Bit specifies
                                   checksum config */
        uint32_t ProtocolId;    /* Protocol Id - ISO9141 / 14230 */   // KWP_ch
} ISO9141_14230_Init_S;

/* ISO9141_14230 Init Link Structure */
typedef struct {
	uint16_t Length;	/* Length of the data bytes */
	uint8_t Data[255];	/* Address byte in case of 5 Baud
							   or data bytes in case of Fast
							   Init */
	uint8_t InitType;	/* 5 Baud or Fast Init */

} ISO9141_14230_LinkInit_S;

/* ISO9141_14230 Init Link Return Structure */
typedef struct {
	uint32_t Timestamp;	/* Rx timestamp */
	uint16_t Length;	/* Length of the data bytes */
	uint8_t ProtocolId;	/* Protocol Id */
	uint8_t IOCtlId;	/* Fast Init or 5 Baud */
	uint8_t Data[255];	/* Key Bytes and Address byte
							   inverse in case of 5 Baud or
							   Fast Init ECU Response data */
} ISO9141_14230_LinkInitRet_S;


/* ISO9141_14230 Rx Message Structure */
typedef struct {
	uint32_t Timestamp;	/* Timestamp for the received msg */
	uint32_t Flags;		/* Rx flags */
	uint16_t Length;	/* Rx message length */
	uint8_t Data[255];	/* Rx data bytes */

} ISO9141_14230_RxMsg_S;



/* ISO9141_14230 Tx message structure */
typedef struct {
	uint32_t Timestamp;	/* Timestamp for the received msg */
	uint32_t Flags;		/* Tx flags */
	uint16_t Length;	/* Tx message length */
	uint8_t Data[255];	/* Tx data bytes */

} ISO9141_14230_TxMsg_S;

extern ISO9141_14230_TxMsg_S ISO9141_14230_TxMsg_S_Buffer;


/* Command Structure */
typedef struct {
	uint32_t *pData;	/* Read / Write Address of the
				   command data for the
				   corresponding IOCtl */
	uint16_t Length;	/* Length of the Command data */
	uint8_t IOCtlId;	/* IOCtl Id */
	uint8_t ParamId;	/* Parameter Id for the IOCtl */

} ISO9141_14230_Cmd_S;




/* enum for ISO9141_14230 return code */
typedef enum {
	NO_ERROR,
	INVALID_COMMAND,
	INVALID_PARAMETERID,
	ISO9141_14230_TXQ_FULL,
	ISO9141_14230_RXQ_EMPTY
} ISO9141_14230_RETCODE;

typedef enum {
	//ADDR_BYTE_WAIT,
	PATTERN_BYTE_WAIT,
	KB1_WAIT,
	KB2_WAIT,
	INV_KB2_WAIT,
	INV_ADDR_WAIT,
	P1MAX_WAIT,
	P3MIN_WAIT,
	P3MIN_P1MAX_WAIT,
	P2_MAX_TIME_WAIT,
	P4MIN_WAIT
}TIMER_WAIT;
typedef struct {
	uint32_t timer_param;
	TIMER_WAIT wait_param_e;

}timeout_param_s;

/*queue buffer */
typedef enum {
	ISO9141_14230_TX_Q,
	ISO9141_14230_RX_Q,
	ISO9141_14230_LEN_Q,
	ISO9141_14230_TIME_Q
} ISO9141_14230_QTYPE;


typedef struct {

	GIOChannel *channel;
}UARTContext;

int kwp_init(void);

void KWP_reset_TimeOut(void);
void KWP_Set_TimeOut(uint32_t , TIMER_WAIT );
void KWP_TimeOut_handle( void );
gboolean KWP_Timer_Handler(gpointer );

void ISO9141_14230_RxTask(void);
ISO9141_14230_RETCODE ISO9141_14230_WriteMsg(void);

void ISO9141_14230_Init(const ISO9141_14230_Init_S *);
ISO9141_14230_RETCODE ISO9141_14230_Command(ISO9141_14230_Cmd_S*);
static void ISO9141_14230_LinkInit(void);
static uint8_t ISO9141_14230_GetChecksum(const uint8_t * , uint16_t , uint8_t);
static bool ISO9141_14230_GetParity(void);

void App_FirstByteRxd(uint8_t , uint32_t );
void PassThruReadMsgResp_KWP (void);

#endif /* _CAN_IF_H_ */

