


#include	<unistd.h>
#include	<glib.h>
#include	"kwp_if.h"
#include        "hfcp.h"


#define FALSE 0
#define TRUE  1


/******************************************************************************
*                   P R I V A T E   V A R I A B L E S
*******************************************************************************/

UARTContext *Global_uart_ctx = NULL;
struct T_GPIO_context	GPIO_KWP_TXD;
struct T_GPIO_context	GPIO_KLINE_LLINE_SLCT;
struct T_GPIO_context	GPIO_KLINE_PULLUP_SLCT;
struct T_GPIO_context	GPIO_LLINE_ISO_SLCT;
struct T_GPIO_context	GPIO_KLINE_ISO_SLCT;
struct T_GPIO_context	GPIO_KW_TX_5BAUD;

/* Init Link Information */
static volatile ISO9141_14230_LinkInit_S l_ISO9141_14230LinkInit_S;

/* Init Link Return information */
static volatile ISO9141_14230_LinkInitRet_S l_ISO9141_14230LinkInitRet_S;

/* ISO 9141 / 14230 Parameters */

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
static uint32_t l_TINIL;	/* Low time for Fast init pattern */
static uint32_t l_TWUP;		/* Wakeup time for Fast init pattern */
static uint32_t l_BaudRate;	/* Baud rate for UART */
static volatile uint16_t l_UARTIntrTxIndex;	/* UART Interrupt Tx length calculation */
static volatile uint16_t l_RxLength;	/* Receive message length */
static volatile uint16_t l_TxLength;	/* Transmit message length */
static volatile uint16_t l_TxIndex;	/* Tx byte index */
static uint32_t l_ProtocolId;	/* Protocol Id */
static uint8_t l_Parity;	/* Parity configuration */
static uint8_t l_DataBits;	/* Number of data bits */
static uint8_t l_FiveBaudMod;	/* Five baud mode */
static volatile uint8_t l_InitStatus;	/* Status of Initialization of link */
static volatile uint8_t l_BitPos;	/* Bit position for 5 Baud or Fast Init */
static uint8_t l_ConnectFlags;	/* Connect Flags */
static uint16_t l_LengthByte;	/* Length byte index */
static uint16_t l_PktLength;	/* Defines the packet length of Handling data wrt length */
static uint32_t l_P1MAX_ExtendedTimeout;	/* Extended P1Max to find the maximum timeout for P1_Max */


/* Queue variables */


static volatile bool l_P3MINTimeout;	/* P3Min timeout indication */
static volatile bool l_MsgReceived;	/* Message Received Indication */
static volatile bool l_TxComp;	/* Tx completion indication */
static uint8_t l_InitTimer;	/* ISO 9141 / 14230 Timer for
				   Initialization */

/* FreeRTOS's returned timer handler, used for uart time guard and receive time out */

static int baudrate;
static int dataLength;
const char *devicePath;
static uint32_t ISO_9141_OR_14230 = 0; // kwp_ch
static uint32_t timer_counter = 0;    //kwp_add

/* KWP Timer handler variables */
static bool timer_handler_flag;
static bool timer_init_flag = FALSE;
timeout_param_s timeout_param_s_var;

static bool fastinit_start_comm_req_start_flag = FALSE;
static bool fastinit_start_comm_req_end_flag = FALSE;

static bool l_p4min_timeout = false;



static ISO9141_14230_QTYPE buffer_type;
static uint16_t kwp_buffer_length;

/* RxMsg and TxMsg handle variable */
bool kwp_RxMsg_received = FALSE;
bool kwp_RxMsg_buffer_updated = FALSE;
bool kwp_TxMsg_received = FALSE;

ISO9141_14230_RxMsg_S ISO9141_14230_RxMsg_S_Buffer;

/* extern variable define here */
ISO9141_14230_TxMsg_S ISO9141_14230_TxMsg_S_Buffer;

/* PassThruReadMsgResp_KWP variable */
static uint8_t l_KWPRX_SegTrnsfr=0;




void KWP_reset_TimeOut(void)
{
	timer_handler_flag = false;
	timer_counter = 0;
}

void KWP_Set_TimeOut(uint32_t Timer_variable,TIMER_WAIT timer_wait_var )
{
	timer_handler_flag = true;
	timeout_param_s_var.timer_param  = Timer_variable;
	timeout_param_s_var.wait_param_e = timer_wait_var; 
}

void KWP_TimeOut_handle(void) 
{
	uint8_t txData;
	static uint16_t data_indx = 0;
	
	if(l_InitStatus == LINKINIT_PENDING)
	{
		if(l_ISO9141_14230LinkInit_S.InitType == FIVE_BAUD_INIT)
		{
			if (timer_handler_flag == true)
			{
				if(timeout_param_s_var.timer_param >= TIME_COUNTER_TO_MS)
				{
					/* print error code */
					switch (timeout_param_s_var.wait_param_e){

					case PATTERN_BYTE_WAIT:
							/* pattern byte error */
							
							/* reset timer_count */
							KWP_reset_TimeOut();
							
							l_RxLength = 0;
							l_TxComp = !FALSE;
							l_P3MINTimeout = !FALSE;
							l_InitStatus == LINKINIT_FAIL;
							
							/* reset the five baud idle time flag */
							timer_init_flag = false;
							
							// error handler
							App_ErrHandler(l_ProtocolId,PATTERN_BYTE_TIMEOUT);
							break;
					case KB1_WAIT:
							/*KB1 error*/
							/* reset timer_count */
							KWP_reset_TimeOut();
							
							l_RxLength = 0;
							l_TxComp = !FALSE;
							l_P3MINTimeout = !FALSE;
							
							l_InitStatus == LINKINIT_FAIL;
							
							/* reset the five baud idle time flag */
							timer_init_flag = false;
							
							// error handler
							App_ErrHandler(l_ProtocolId, KEY_BYTE1_TIMEOUT);
							break;
					case KB2_WAIT:
							/*KB2 error*/
							/* reset timer_count */
							KWP_reset_TimeOut();
							
							l_RxLength = 0;
							l_TxComp = !FALSE;
							l_P3MINTimeout = !FALSE;
	
							l_InitStatus == LINKINIT_FAIL;
							
							/* reset the five baud idle time flag */
							timer_init_flag = false;
							
							// error handler
							App_ErrHandler(l_ProtocolId, KEY_BYTE2_TIMEOUT);
							break;
					case INV_KB2_WAIT:
							
							 /* sent tx msg */ 
							if((l_RxLength == ADDRESS_BYTE_INV_POS) && (l_TxLength != 0))
							{
								txData = l_ISO9141_14230LinkInitRet_S.Data[2] ^ 0xFF;
							
								// call tx sent msg function
								uart_send_data(Global_uart_ctx,(char)txData);
								
								if(l_FiveBaudMod == FIVEBAUD_MODE_ZERO)
								{
									/* reset timer_count */
									KWP_reset_TimeOut();
									KWP_Set_TimeOut(W4_MAX_TIME,INV_ADDR_WAIT);
									
								}
								else
								{
									/* do nothing */
								}
							}
							else
							{
								/*do nothing */
							}
							break;
					case INV_ADDR_WAIT:        		
							/*inverse addr byte error */
							/* reset timer_count */
							KWP_reset_TimeOut();
							
							l_RxLength = 0;
							l_TxComp = !FALSE;
							l_P3MINTimeout = !FALSE;
	
							l_InitStatus == LINKINIT_FAIL;
							
							/* reset the five baud idle time flag */
							timer_init_flag = false;
							
							// error handler
							App_ErrHandler(l_ProtocolId, ADDRESS_BYTE_TIMEOUT);
							
							break;
					default:
							break;
					}
				}
				else
				{
					//do nothing
				}
			}
			else
			{
				/*do nothing */
			}
		}
		/* FAST INIT */
		else
		{
			if (timer_handler_flag == true)
			{
				if(timeout_param_s_var.timer_param >= TIME_COUNTER_TO_MS)
				{	
					/* Check if length is nonzero. If no bytes are received then timeout on Fast Init ECU response else indicate end of Fast Init ECU response */
					if((timeout_param_s_var.wait_param_e != P3MIN_WAIT)||(timeout_param_s_var.wait_param_e != P4MIN_WAIT)){
					
						if (l_RxLength != 0) {

							/* End of frame indication */

							/* Update the length information to the Length Queue */
							
							kwp_buffer_length = l_RxLength;

							/* Reset Rx Length */
							l_RxLength = 0;

							/* Indicate Message received */
							l_MsgReceived = !FALSE;

							/* If P3 min is less than P1max then next Tx could be started immediately else wait for P3min - P1max */
							if (l_P3MIN < l_P1MAX) {

								/* Update the status of P3min timeout */
								l_P3MINTimeout = !FALSE;
								
							}

							else {
								/* should update the l_InitStatus = LINKINIT_DONE before elasped the time*/
								
								/* reset timer_count */
								KWP_reset_TimeOut();
								KWP_Set_TimeOut((l_P3MIN - l_P1MAX),(P3MIN_P1MAX_WAIT));
							}
						}

						else {

							/* Timeout on Fast Init ECU reponse */
							/* Call application error handler */
						        App_ErrHandler(l_ProtocolId,FASTINIT_RESP_TIMEOUT);

							/* Set Link Init failure */
							l_InitStatus = LINKINIT_FAIL;

							/* Update the status of P3min timeout - Could load the Tx bytes */
							l_P3MINTimeout = !FALSE;
						}
					}
					/* send start communication service request message */
					else{
						if(data_indx < l_TxLength ){
							txData = l_ISO9141_14230LinkInitRet_S.Data[data_indx] ;
							uart_send_data(Global_uart_ctx,(char)txData);
							data_indx++;
							KWP_reset_TimeOut();
							KWP_Set_TimeOut((l_P4MIN),(P4MIN_WAIT));
						}
						else{
						
						}
						if(data_indx == l_TxLength )
						{
							fastinit_start_comm_req_end_flag = !FALSE;
							
						}
						else
						{
							/* Do nothing */
						}
						
						
					}
				}
				else
				{
						/* do nothing */
				}
			}
		}
	}
	/* Normal operation - Timeout */
	else if((l_InitStatus == LINKINIT_DONE) || (l_InitStatus == NO_LINKINIT) || (l_InitStatus == LINKINIT_FAIL))
	{
		if (timer_handler_flag == true)
		{
			if(timeout_param_s_var.timer_param >= TIME_COUNTER_TO_MS)
			{
				if(timeout_param_s_var.wait_param_e != P4MIN_WAIT )
				{
					if (l_MsgReceived == FALSE) 
					{
						if (l_RxLength != 0) 
						{
							if (KWP_PROTOCOL_ID == 0x04 /*get_ISO9141_or_14230()*/) 
							{
								if (l_RxLength != l_PktLength) 
								{
								
								/* needs to do error handler */
									App_ErrHandler(l_ProtocolId,ECU_RESP_TIMEOUT); 
								#if 0
									l_P1MAX_ExtendedTimeout += l_P1MAX;
									if (l_P1MAX_ExtendedTimeout < P1_MAX_EXTENDED_TIMEOUT) {

										/* Load the RxTimeout to determine end of frame */
										timer_handler_flag = false;
										timer_counter = 0;
										KWP_Set_TimeOut(l_P1MAX,P1MAX_WAIT);
										return;
									}

									else {
										
										//P1_MAX_WAIT_TIMEOUT has occoured, continue with the normal flow
										timer_handler_flag = false;
										timer_counter = 0;								
									}
								#endif
								}
							}
							
			#if 0
							/* P1MAX time restart */
							timer_handler_flag = false;
							timer_counter = 0 ;
							
							/* End of frame indication */
							l_P1MAX_ExtendedTimeout = 0;

						

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
						//		UART_Change_RxTimeOut (UART_CH_ID_1, 0);
							}

							else {
								KWP_Set_TimeOut((l_P3MIN - l_P1MAX),P3MIN_WAIT);
						//		UART_Change_RxTimeOut (UART_CH_ID_1,(l_P3MIN - l_P1MAX));
						//		UART_Restart_RxTimeOut (UART_CH_ID_1);
							}
			#endif
						}
		
						else {

						/* Timeout on ECU reponse for Tester request */
						/* Call application error handler */
					        App_ErrHandler(l_ProtocolId,ECU_RESP_TIMEOUT); 

						/* Minimum time P3min has elapsed. Next Tx can be started */
						l_P3MINTimeout = !FALSE;
						/* reset timer_count */
						KWP_reset_TimeOut();					
						}
					}

					else {

					/* Minimum time P3min has elapsed. Next Tx can be started */
					l_P3MINTimeout = !FALSE;
			
					/* reset timer_count */
					KWP_reset_TimeOut();
			
				     	}
				}
				else
				{
					l_p4min_timeout = !FALSE;
					KWP_reset_TimeOut();	
				}
			}
		
		}	
	}

	else
	{
		/*Do nothing*/
	}
}

gboolean KWP_Timer_Handler(gpointer data)
{
	uint32_t fl_CurrTime;
	uint16_t fl_Cnt;
	uint8_t fl_ChkSum = CHECKSUM_OK;
	/* Determine the status of Init Link */
	if (l_InitStatus == LINKINIT_PENDING) {

		timer_counter++;

		/* Determine the type of initialization */
		if (l_ISO9141_14230LinkInit_S.InitType == FIVE_BAUD_INIT) {

			/* 5 Baud Initialization */

			/* Check if the Bus was idle for W5 or W0. If idle perform 5 Baud initialization else wait till W5 or W0 */
			if(TIME_COUNTER_TO_MS == l_5BaudIdleTime) { 
				timer_init_flag = true;
			    //timer_counter = 0;
			    
			}
			if (timer_init_flag == true) {

				/* Start bit */
				if (l_BitPos == START_BIT_POS) {

					/* Disable bus monitoring for Idle time. Idle time is
					   achieved so no more updation of Ref Idle time */
					//ISO9141_14230_DisableBusMonitor();

					/* Send the Start Bit */
		  	                 T_gpio_PIN_set(&GPIO_KWP_TXD, 0);                               

					/* Update the Init reference time */
					timer_counter = 0;

					/* Increment Bit position to Transmit the next bit */
					l_BitPos++;
				}

				/* Send Address byte */

				/* Check if the Bit position neither in Start or Stop bit position and also check if FIVE Baud time has elapsed since
				   the transmission of Start bit */
				else if ((l_BitPos > START_BIT_POS) && (l_BitPos < STOP_BIT_POS) && (TIME_COUNTER_TO_MS >= FIVE_BAUD_TIME)) {

					/* Transmit address bits till the Bit position reaches MSB */
					if (l_BitPos <= l_DataBits) {

						/* Address decoding - Check if the bit is 0 or 1 */
						if (CHECK_BITU8(l_ISO9141_14230LinkInit_S.Data[0],(l_BitPos - 1)) != FALSE) {

							/* Bit is 1 - Set K Line */
							T_gpio_PIN_set(&GPIO_KWP_TXD, 1); 
						}

						else {

							/* Bit is 0 - Reset K Line */
			                               T_gpio_PIN_set(&GPIO_KWP_TXD, 0); 
						}

						/* Increment the bit position to transmit the next bit */
						l_BitPos++;

						/* Update Reference Init time to current time */
						timer_counter = 0;
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

						/* Based on the Parity configuration determine if the Parity bit is 0 or 1 */
						if (ISO9141_14230_GetParity() != FALSE) {

							/* Parity Bit is 1 - Set K Line */
							T_gpio_PIN_set(&GPIO_KWP_TXD, 1); 
						}

						else {

							/* Parity Bit is 0 - Reset K Line */
				                        T_gpio_PIN_set(&GPIO_KWP_TXD, 0); 
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
						timer_counter = 0;
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
				if ((l_BitPos == STOP_BIT_POS) && (TIME_COUNTER_TO_MS >= FIVE_BAUD_TIME)) {

					/* Stop Bit */
			                T_gpio_PIN_set(&GPIO_KWP_TXD, 1); 

					/* Update the Init reference time */			
					timer_counter = 0;
					 
					/* Increment bit position to indicate no more bits - All
					   bits are sent */
					l_BitPos++;
				}

				/* Perform operations after transmission of Stop bit */
				else if ((l_BitPos == (STOP_BIT_POS + 1)) && (TIME_COUNTER_TO_MS >= FIVE_BAUD_TIME)) {

					/* Check if L line was configured. If yes then disable */
					if (CHECK_BITU8(l_ConnectFlags, BIT_LINECONF) == FALSE) {

						/* Disable L line after Initialization ?????? */
				                T_gpio_PIN_set(&GPIO_KLINE_LLINE_SLCT, 0); 
					}

					/* Configure the UART Tx as Peripheral */
					// Config_Pin_UART_Mode(KW_TXD);

					/* reset timer_count */
					KWP_reset_TimeOut();
					KWP_Set_TimeOut(l_W1, PATTERN_BYTE_WAIT);
					

					/* Increment bit position */
					l_BitPos++;
				}

				/* Check for Rx bytes from ECU after completion of Stop bit */
				else if (l_BitPos > (STOP_BIT_POS + 1)) {

					/* getting length from buffer */
					l_ISO9141_14230LinkInitRet_S.Length = kwp_buffer_length ;
					
					if (l_ISO9141_14230LinkInitRet_S.Length != 0) {
						/* Check if Length has reached Address byte position
						   - Key bytes received */
						if (l_ISO9141_14230LinkInitRet_S.Length == ADDRESS_BYTE_INV_POS) {

							/* Obtain Rx data upto the frame Length from the
							   UART */
							/*  UART_MidRecv, uartMidRecv these functions are moved to UART_Handler */ 
							
							/* Based on the 5 Baud configuration perform operations upon reception of Key bytes */
							if ((l_FiveBaudMod == FIVEBAUD_MODE_ZERO) || (l_FiveBaudMod == FIVEBAUD_MODE_ONE))
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
								
								/* reset timer_count */
								KWP_reset_TimeOut();
								KWP_Set_TimeOut(l_W4 - 19500, INV_KB2_WAIT);
								
								
							}

							else if (l_FiveBaudMod == FIVEBAUD_MODE_TWO)
							{

								/* In this Mode, receive ECU Pattern and
								   Key bytes and receive the Address inverse
								   - No transmission of Key byte 2 inverse */

								/* Clear the Tx length to indicate no Tx byte */
								l_TxLength = 0;

								/* Load Max Rx timeout for Address inverse byte
								   W4 - RETTO */
								
								/* reset timer_count */
								KWP_reset_TimeOut();
								KWP_Set_TimeOut(W4_MAX_TIME, INV_ADDR_WAIT);
							}

							else if (l_FiveBaudMod == FIVEBAUD_MODE_THREE)
							{

								/* In this Mode, receive ECU Pattern and
								   Key bytes - No Tx of Key byte 2 inverse and
								   no Rx of address inverse byte */

								/* Clear the Tx length to indicate no Tx byte */
								l_TxLength = 0;

								/* Clear Rx Length */
								l_RxLength = 0;

								/* Store Protocol Id */
								l_ISO9141_14230LinkInitRet_S.ProtocolId = l_ProtocolId;

								/* Store Init type */
								l_ISO9141_14230LinkInitRet_S.IOCtlId = FIVE_BAUD_INIT;

								/* Store Only Key Bytes - Store the Keybytes at 0th and 1st element of the array and modify
								   length to indicate only Keybytes */
								l_ISO9141_14230LinkInitRet_S.Data[0] = l_ISO9141_14230LinkInitRet_S.Data[1];
								l_ISO9141_14230LinkInitRet_S.Data[1] = l_ISO9141_14230LinkInitRet_S.Data[2];
								l_ISO9141_14230LinkInitRet_S.Length = 2;

								/* Report to the Call Back function */
								App_InitData((void *)&l_ISO9141_14230LinkInitRet_S);

								/* Set the Link Init Status to Done */
								l_InitStatus = LINKINIT_DONE;
								
								/* reset the five baud idle time flag */
							        timer_init_flag = false;
								
								/* Stop Timer for Initialization for ISO 9141/14230 */
								//stop_time_stamp(l_InitTimer);

								/* Indicate response bytes received */
								l_MsgReceived = !FALSE;

								/* Load P3 min timeout for minimum time before Data transmission */
								/* reset timer_count */
								KWP_reset_TimeOut();
								KWP_Set_TimeOut(l_P3MIN, P3MIN_WAIT);
							}

							else {

								/* Should not enter else */
							}
							
							/* length reset */
							kwp_buffer_length = 0;
							
						}

						/* Check if length has exceeded Address byte position
						   - Address byte inverse received */
						else if
						    (l_ISO9141_14230LinkInitRet_S.Length > ADDRESS_BYTE_INV_POS) {

							/* Store the Address byte inverse after Key bytes */
							/* UART_MidRecv, uartMidRecv These functions are  moved to UART handler*/
						
							/* Based on the 5 Baud configuration perform
							   operations upon reception of Address inverse
							   byte */
							if (l_FiveBaudMod == FIVEBAUD_MODE_ZERO || l_FiveBaudMod == FIVEBAUD_MODE_TWO) {

								/* In this FIVEBAUD_MODE_ZERO, receive ECU
								   Pattern and Key bytes, Transmit KeyByte
								   inverse and then Receive Address Inverse
								   byte */

								/* In this FIVEBAUD_MODE_TWO, receive ECU
								   Pattern and Key bytes and receive the
								   Address inverse - No transmission of
								   Key byte 2 inverse */

								/* Store Protocol Id */
								l_ISO9141_14230LinkInitRet_S.ProtocolId = l_ProtocolId;

								/* Store Init type */
								l_ISO9141_14230LinkInitRet_S.IOCtlId = FIVE_BAUD_INIT;

								/* Store Only Key Bytes - Store the Keybytes at 0th and 1st element of the array and modify
								   length to indicate only Keybytes */
								l_ISO9141_14230LinkInitRet_S.Data[0] = l_ISO9141_14230LinkInitRet_S.Data[1];
								l_ISO9141_14230LinkInitRet_S.Data[1] = l_ISO9141_14230LinkInitRet_S.Data[2];
								l_ISO9141_14230LinkInitRet_S.Length = 2;

								/* Report to the Call Back function */
								App_InitData((void *)&l_ISO9141_14230LinkInitRet_S);


								/* Set the Link Init Status to Done */
								l_InitStatus = LINKINIT_DONE;
								timer_init_flag = false;

								/* Indicate response bytes received */
								l_MsgReceived = !FALSE;

								/* Load P3 min timeout for minimum time before Data transmission */
								
								/* reset timer_count */
								KWP_reset_TimeOut();
								KWP_Set_TimeOut(l_P3MIN, P3MIN_WAIT);
								
								/* length reset */
								kwp_buffer_length = 0;
								
							}
						}
						else
						{
						/* do nothing */
						}
						
					}
					else
					{
						/* Do nothing */
					}
				}

				else {

					/* Do nothing */
				}
			}
			
		   }
		

		else {

			/* Fast Init */
	
			/* Check if the Bus was idle for TIdle. If idle perform Fast initialization else wait till TIdle */
			if (TIME_COUNTER_TO_MS >= l_TIDLE) {
				timer_init_flag = true;
			}
			if(timer_init_flag == true) {

					/* Generate Wakeup Pattern */
					if (l_BitPos == WKUP_TINIL) {

						/* Disable bus monitoring for Idle time. Idle time is
						   achieved so no more updation of Ref Idle time */
						/* ISO9141_14230_DisableBusMonitor(); */

						/* TiniL pattern */
						T_gpio_PIN_set(&GPIO_KWP_TXD, 0);
						
						/* Update the Init reference time */
						timer_counter = 0;

						/* Increment l_BitPos to perform rest of the pattern */
						l_BitPos++;
					}

					else if ((l_BitPos == WKUP_TWUP) && (TIME_COUNTER_TO_MS >= l_TINIL)) {

						/* TWup - TiniL pattern */
						T_gpio_PIN_set(&GPIO_KWP_TXD, 1);
						/* Time not updated to represent the rest of Twup */
						/* Increment l_BitPos to indicate end of wakeup pattern */
						l_BitPos++;
					}

					else if (((l_BitPos == WKUP_STARTREQ) && (TIME_COUNTER_TO_MS >= l_TWUP)) || (fastinit_start_comm_req_start_flag =! FALSE)){

						/* Check if L line was configured. If yes then disable */
						if (CHECK_BITU8(l_ConnectFlags, BIT_LINECONF) == FALSE) {

							/* Disable L line after Initialization ?????? */
							T_gpio_PIN_set(&GPIO_KLINE_LLINE_SLCT, 0);
						}

						
						/* Configure the Receive time out for UART - RETTO */
						/* If P3 min is less than P1max then limit the time to P1max as P1 max is the minimum time to determine the
						   end of frame else load P3min */
						if (l_P3MIN < l_P1MAX) {
							
							/* reset timer_count */
							KWP_reset_TimeOut();
							KWP_Set_TimeOut(l_P1MAX, P1MAX_WAIT);
							
						}

						else {
							
							/* reset timer_count */
							KWP_reset_TimeOut();
							KWP_Set_TimeOut(l_P3MIN, P3MIN_WAIT);
						}

						/* Configure Tx Timeguard */
						//UART_Change_TimeGuard(UART_CH_ID_1, l_P4MIN);    //need to check

						//AMIT
						//UART_Change_TimeGuard(UART_CH_ID_1, 1);

						/* Set the Tx length to indicate Tx bytes */
						l_TxLength = l_ISO9141_14230LinkInit_S.Length;

				#if 0
						/* Load UART with the data bytes */
						for (fl_Cnt = 0; fl_Cnt < l_ISO9141_14230LinkInit_S.Length; fl_Cnt++) {
							if (0 == fl_Cnt) {

								//uartMidSendFirstByte(UART_CH_ID_1,l_ISO9141_14230LinkInit_S.Data[fl_Cnt]);
								//uartMidSendFirstByte(UART_CH_ID_1,(uint8_t *) &l_ISO9141_14230LinkInit_S.Data[fl_Cnt]);
								//UART_Change_TimeGuard(UART_CH_ID_1, l_P4MIN);
							}

							else {

								//uartMidSend(UART_CH_ID_1,
								//(uint8_t *)&l_ISO9141_14230LinkInit_S.Data[fl_Cnt], 1);
							}
						}

						/* Increment l_BitPos to Receive response bytes */
						l_BitPos++;
				#endif
				
						fastinit_start_comm_req_start_flag = !FALSE;
						if(fastinit_start_comm_req_end_flag = !FALSE)
						{
							l_BitPos++;
							fastinit_start_comm_req_end_flag   = FALSE;
							fastinit_start_comm_req_start_flag = FALSE;
							
						}
						else
						{
						}
						
					}

					else if (l_BitPos > WKUP_STARTREQ) {

						/* Check if ECU response has been received based on RxLength information */
						//if (ISO9141_14230_DelFromQ(ISO9141_14230_LEN_Q, (void *)&l_ISO9141_14230LinkInitRet_S.Length)!= ISO9141_14230_Q_EMPTY) {
							l_ISO9141_14230LinkInitRet_S.Length = kwp_buffer_length;
							if(l_ISO9141_14230LinkInitRet_S.Length != 0) {

							/* Obtain bytes from the UART */
							/* UART_MidRecv, uartMidRecv these functions are moved to UART handler */
							
							/* Verify checksum if configured */
							if (CHECK_BITU8(l_ConnectFlags,BIT_CHKSUM) == FALSE) {

								/* Obtain the checksum for Init data */
								fl_ChkSum = ISO9141_14230_GetChecksum((void *)l_ISO9141_14230LinkInitRet_S.Data, l_ISO9141_14230LinkInitRet_S.Length, CHKSUM_RXDATA);

								/* If Checksum is ok */
								if (fl_ChkSum == CHECKSUM_OK) {

									/* Update length */
									l_ISO9141_14230LinkInitRet_S.
									    Length--;

									/* Remove the checksum byte */
									l_ISO9141_14230LinkInitRet_S.Data [l_ISO9141_14230LinkInitRet_S.Length]= 0;

									/* Set the status of Fast Init */
									l_InitStatus = LINKINIT_DONE;
									
									/*reset init timer flag*/
									timer_init_flag = false;

									/* Update the Timestamp */
									//ISO9141_14230_DelFromQ(ISO9141_14230_TIME_Q,(ISO9141_14230_Q_S*)&l_ISO9141_14230LinkInitRet_S.Timestamp);

									/* Store Protocol Id */
									l_ISO9141_14230LinkInitRet_S.ProtocolId = l_ProtocolId;

									/* Store Init type */
									l_ISO9141_14230LinkInitRet_S.IOCtlId = FAST_INIT;

									/* Report to the Call Back function */
									App_InitData((void *)&l_ISO9141_14230LinkInitRet_S);
								}

								/* Report Checksum error */
								else {

									/* Set the status of Fast Init */
									l_InitStatus = LINKINIT_FAIL;
									
									/*reset init timer flag*/
									timer_init_flag = false;
									

									/* Call application error handler */
									App_ErrHandler(l_ProtocolId,ECU_RESPCHKSUM_ERROR);
								}

								/* Stop Timer for Initialization for ISO 9141/14230 */
								//  stop_time_stamp(l_InitTimer);
							}

							/* If no checksum configured */
							else {

								/* Set the status of Fast Init */
								l_InitStatus = LINKINIT_DONE;
								
								/*reset init timer flag*/
								timer_init_flag = false;

								/* Store Protocol Id */
								l_ISO9141_14230LinkInitRet_S.
								    ProtocolId =
								    l_ProtocolId;

								/* Store Init type */
								l_ISO9141_14230LinkInitRet_S.
								    IOCtlId = FAST_INIT;

								/* Report to the Call Back function */
								App_InitData((void *)&l_ISO9141_14230LinkInitRet_S);

								/* Stop Timer for Timestamp for ISO 9141/14230 */
				                                //stop_time_stamp(l_InitTimer);
						}
						
					kwp_buffer_length = 0;               // doubt, not sure.
					}

					else {

						/* Do nothing */
					}
				}

			}
		}
	}

	else {

		/* Do nothing */
	}
	/* check TimeOut periodically */
	KWP_TimeOut_handle();
	
	/* check response sent periodically */
	PassThruReadMsgResp_KWP ();
	
	/*check TxTask periodically*/
	ISO9141_14230_TxTask();
	
	/*check RxTask periodically*/
	ISO9141_14230_RxTask();
	
	return 1;

}

gboolean KWP_uart_rx_callback(GIOChannel *source, GIOCondition condition, gpointer data)
{
	if(condition & G_IO_IN)
	{
		uint16_t fl_Cnt;
		uint32_t fl_CurrTime = 0;
		uint8_t fl_UARTRxByte, fl_TxByte;
		gchar l_buffer_recev = 0;

		/* UART_Receive */
		
		gsize read_byte;
		GError *error = NULL;
		

		GIOStatus status = g_io_channel_read_chars(source, &l_buffer_recev ,1, &read_byte,&error);

		if(error)
		{
			//gprint("Error");
			return;
		}

		if(status == G_IO_STATUS_NORMAL && read_byte > 0  )
		{
			/*Byte read*/
			/*copy the data to necessary buffer*/
			
			
		}



		/* Update current time */
		/* Based on the interrupt type perform operations */
		/* Rx Interrupt */
		//if (CHECK_BITU8(p_IntrType, RX_INTR) != FALSE) //ch 

			/* Check if Tx length is zero. If non zero then byte obtained is the
			   byte transmitted */
			if (l_TxLength != 0) {


				/* Determine the Tx byte from the type of Tx data */

				/* Check if Init Tx Data */
				if (l_InitStatus == LINKINIT_PENDING) {

					/* Determine the type of Init */

					/* 5 Baud Init */
					if (l_ISO9141_14230LinkInit_S.InitType == FIVE_BAUD_INIT) {

						/* Tx byte is the Keybyte 2 inverse byte */
						fl_TxByte = l_ISO9141_14230LinkInitRet_S.Data[2] ^ 0xFF;
					}

					else {

						/* Tx byte is the Init data byte */
						fl_TxByte = l_ISO9141_14230LinkInit_S.Data [l_UARTIntrTxIndex];
					}
				}

				/* Normal Tx data byte */
				else {

					/* Retrieve Tx data byte from Tx Queue */
		                        //ISO9141_14230_GetByteFromQ (ISO9141_14230_TX_Q, l_UARTIntrTxIndex, &fl_TxByte);  call send TX msg function
					
					/* check send msg, How tx buffer updated ? and needs to check where will clear buffer*/
					buffer_type = ISO9141_14230_TX_Q;
			//		fl_TxByte   = kwp_tx_buffer[0];
					fl_TxByte   = ISO9141_14230_TxMsg_S_Buffer.Data[l_UARTIntrTxIndex];
				}

				/* Check for Corruption on the bus */

				/* Compare Received byte with Transmitted byte */
	
	
				fl_UARTRxByte =	(uint8_t)l_buffer_recev;
				
				if (fl_UARTRxByte != fl_TxByte) {

					/* Corruption on the Bus */
					/* Abort Transmission - Flush buffers */

					/* Call application error handler */
					App_ErrHandler(l_ProtocolId, MSG_CORRUPTED); 

					/* If Tx byte is from Initialization set status to failure */
					if (l_InitStatus == LINKINIT_PENDING) {
					
						l_InitStatus = LINKINIT_FAIL;
						timer_init_flag = false ;
						
						/* Stop Timer for Initialization for
						   ISO 9141/14230 */
						
						// stop_time_stamp(l_InitTimer);
					}

					
					
				//	timer_handler_flag = false; // check it's need .
				//	timer_counter = 0;

					/* reset timer_count */
					KWP_reset_TimeOut();
					
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
		                    UART_Change_RxTimeOut(AT91C_BASE_US0, l_BaudRate, l_P1MAX);
		                    UART_Restart_RxTimeOut(AT91C_BASE_US0);

		                }
		                else
		                {

		                    AT91C_BASE_US0->US_IER =0x00000100;
		                    UART_Change_RxTimeOut(AT91C_BASE_US0, l_BaudRate,l_P3MIN);
		                    UART_Restart_RxTimeOut(AT91C_BASE_US0);
		                }*/
							KWP_reset_TimeOut();
							KWP_Set_TimeOut(P2_MAX_TIME,P2_MAX_TIME_WAIT);

							/* Update the P3min Timeout indication in order
							   to wait for time P3min before next Txn */
							l_P3MINTimeout = FALSE;

							/* Initialize Message received status */
							l_MsgReceived = FALSE;

						}

						/* For Five Baud Mode 1 indicate Init Done */
						else if ((l_InitStatus == LINKINIT_PENDING) && (l_ISO9141_14230LinkInit_S.InitType == FIVE_BAUD_INIT) && (l_FiveBaudMod == FIVEBAUD_MODE_ONE)) {

							/* In this Mode, receive ECU Pattern and
							   Key bytes, Transmit KeyByte inverse
							   No reception of Address Inverse byte */

							/* Set the Link Init Status to Done */
							l_InitStatus = LINKINIT_DONE;
  							
  							/* reset the five baud idle time flag */
							timer_init_flag = false; 
							
							/* Store Protocol Id */
							l_ISO9141_14230LinkInitRet_S.ProtocolId = l_ProtocolId;

							/* Store Init type */
							l_ISO9141_14230LinkInitRet_S.IOCtlId = FIVE_BAUD_INIT;

							/* Store Only Key Bytes - Store the Keybytes at 0th and 1st element of the array and modify
							   length to indicate only Keybytes */
							l_ISO9141_14230LinkInitRet_S.Data[0] = l_ISO9141_14230LinkInitRet_S.Data[1];
							l_ISO9141_14230LinkInitRet_S.Data[1] = l_ISO9141_14230LinkInitRet_S.Data[2];
							l_ISO9141_14230LinkInitRet_S.Length = 2;

							/* Report Link init done to the Call Back
							   function */
							App_InitData((void *) &l_ISO9141_14230LinkInitRet_S); 

							/* Stop Timer for Timestamp for ISO 9141/14230 */
							// stop_time_stamp(l_InitTimer);

							/* Indicate response bytes received */
							l_MsgReceived = !FALSE;

							/* Load P3 min timeout for minimum time before
							   Data transmission */
						
							/* reset timer_count */
							KWP_reset_TimeOut();
							
							KWP_Set_TimeOut(l_P3MIN, P3MIN_WAIT);  
							
	
							l_P3MINTimeout = FALSE; // doubt
						}
						else if ((l_InitStatus == LINKINIT_PENDING) && (l_ISO9141_14230LinkInit_S.InitType == FAST_INIT))
						{
							KWP_reset_TimeOut();
							KWP_Set_TimeOut(P2_MAX_TIME,P2_MAX_TIME_WAIT);
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
					if (l_ISO9141_14230LinkInit_S.InitType == FIVE_BAUD_INIT) {

						/* Determine the byte received - Load corresponding
						   timeouts */
						if (l_RxLength == PATTERN_BYTE_POS) {
							
							l_ISO9141_14230LinkInitRet_S.Data[l_RxLength] = (uint8_t)l_buffer_recev;
							/* call Uart receive uartMidRecv- moved to uart handler */
							
							/* Received the pattern byte - Increment the length */
							l_RxLength++;

							/* Load the Receive timeout W2 - RETTO */
							/* reset timer_count */
							KWP_reset_TimeOut();
							KWP_Set_TimeOut(l_W2,KB1_WAIT);
							
						}

						else if (l_RxLength == KEY_BYTE1_POS) {

							/* call Uart receive uartMidRecv- moved to uart handler */
							l_ISO9141_14230LinkInitRet_S.Data[l_RxLength] = (uint8_t) l_buffer_recev;
							
							/* Received the Key byte 1 - Increment the length */
							l_RxLength++;

							/* Load the Receive timeout W3 - RETTO */
	
							/* reset timer_count */
							KWP_reset_TimeOut();
							
							KWP_Set_TimeOut(l_W3, KB2_WAIT);                  //ch
							
						}

						else if (l_RxLength == KEY_BYTE2_POS) {

							/* call Uart receive uartMidRecv- moved to uart handler */
							l_ISO9141_14230LinkInitRet_S.Data[l_RxLength] = (uint8_t) l_buffer_recev;
							/* Received the Key byte 2 - Increment the length */
							l_RxLength++;

							/* Update the length information to the Length Queue */
							/* adding length in buffer */
							kwp_buffer_length = l_RxLength;
							

							/* Disable Rx Timeout - For Address inverse byte
							   Rx timeout is loaded based on the 5 Baud mode */
							
							
							/* reset timer_count */
							KWP_reset_TimeOut();                                    //ch
							
						}

						else if (l_RxLength == ADDRESS_BYTE_INV_POS) {

							/* call Uart receive uartMidRecv- moved to uart handler */
							l_ISO9141_14230LinkInitRet_S.Data[l_RxLength] = (uint8_t) l_buffer_recev;
							
							/* Received the Address byte inverse - Increment the
							   length */
							l_RxLength++;

							/* Update the length information to the Length Queue */
							
							/* adding length in buffer */
							kwp_buffer_length = l_RxLength;

							/* Reset Rx Length */
							l_RxLength = 0;

							/* Disable Rx Timeout */
							
							/* reset timer_count */
							KWP_reset_TimeOut();                             //ch
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
							
							/*reset the (P3min +50000 )timer */
							
							/* reset timer_count */
							KWP_reset_TimeOut();
							
							KWP_Set_TimeOut(l_P1MAX, P1MAX_WAIT);  

							/* FIRST_BYTE_RECVD indication ???? */
							/* Call back function */
							/* Commented the First Byte Rx Indication for Fast Init */
							/* RA Consultant recomended code */
							App_FirstByteRxd(l_ProtocolId,fl_CurrTime);

							/* Update Rx timestamp */
							//ISO9141_14230_AddToQ(ISO9141_14230_TIME_Q,(const ISO9141_14230_Q_S*)&fl_CurrTime, 0); // call uart read() 

							/* Initialize Message received status */
							l_MsgReceived = FALSE;
						}
						
						/* Call uart receive */
					        //uartMidRecv(UART_CH_ID_1,(uint8_t *) &l_ISO9141_14230LinkInitRet_S.Data[fl_Cnt], 1);
					
						l_ISO9141_14230LinkInitRet_S.Data[l_RxLength] = (uint8_t) l_buffer_recev;
					
						
						/* Received ECU response byte - Increment length */
						l_RxLength++;

					}
				}

				/* Receive ECU response bytes for Tester request */
				else if ((l_InitStatus == LINKINIT_DONE) || (l_InitStatus == NO_LINKINIT) || (l_InitStatus == LINKINIT_FAIL)) {

					/* If First byte received */
					if (l_RxLength == 0) {
						
						/* reset P2 Max time reset */
						KWP_reset_TimeOut();
						
						l_LengthByte = 0;
						l_PktLength = 0;
						
						/* clear ISO9141_14230_RxMsg_S_Buffer structure */
						memset((void*)&ISO9141_14230_RxMsg_S_Buffer,0,sizeof(ISO9141_14230_RxMsg_S_Buffer));
						
						if(KWP_PROTOCOL_ID == 0x04 /*get_ISO9141_or_14230()*/) {
							l_PktLength = (uint8_t)l_buffer_recev;
						}

						/* Load the RxTimeout to determine end of frame */
					        //KWP_Set_TimeOut(l_P1MAX,P1MAX_WAIT);  // changed P1max time format
	

						/* FIRST_BYTE_RECVD indication ???? */
						/* Call back function */
						App_FirstByteRxd(l_ProtocolId,fl_CurrTime);  // check implementation

						/* Initialize Message received status */
						l_MsgReceived = FALSE;
					}


					else if (l_RxLength == l_LengthByte) {
						if (KWP_PROTOCOL_ID == 0x04 /*get_ISO9141_or_14230()*/) {
							l_PktLength = (uint8_t)l_buffer_recev;
						}
					}

					else {

						//Do nothing    
					}

					/* Received ECU response byte - Increment length */
					

					/* store Rx data in RxMsg_s Buffer */
					ISO9141_14230_RxMsg_S_Buffer.Data[l_RxLength] = (uint8_t) l_buffer_recev;
					
					l_RxLength++;
					
					/* reset timer_count */
					KWP_reset_TimeOut();
					
					/* set P1 MAX for every byte receiving from ECU */
					KWP_Set_TimeOut(l_P1MAX,P1MAX_WAIT); 
					
					
					if (l_RxLength == l_PktLength) {
					
						/* reset timer_count */
						KWP_reset_TimeOut();  // check it's need or not 

						// Complete packet received
						// Handle the timeout functionalities
					
						l_PktLength = 0;
						
						//fl_CurrTime = fl_CurrTime - (l_P1MAX);

			
						/* adding length in buffer */
						kwp_buffer_length = l_RxLength;
						
						kwp_RxMsg_received = !FALSE ;
						
						/* adding length to RxMsg_s Buffer */
						ISO9141_14230_RxMsg_S_Buffer.Length = l_RxLength;
						
						/* Reset Rx Length */
						l_RxLength = 0;

						/* Indicate Message received */
						l_MsgReceived = !FALSE;

						/* Check P3min to load additional timeout starting from P1max */
						/* If P3 min is less than P1max then next Tx could be
						   started immediately else wait for P3min - P1max */
						if (l_P3MIN < l_P1MAX) {

							/* Update the status of P3min timeout */
							l_P3MINTimeout = !FALSE;

							/* Disable Rx timeout */
	                         
	                         			/* reset timer_count */
							KWP_reset_TimeOut();
						}

						else {

							/* - This was an Error which we had face in REVA for Diagnostic Tester Application
							   - We had Changed the implementation of KWP Packet handling based on the Length.
							   - So we were not maintaining the delay of 55 mses for response to request (P3_Min) timeout.
							   - We shouldn't subtract the l_P1MAX timeout from l_P3MIN, (55-20) = 35 msecs delay, which is wrong. 
							 */
							
							/* reset timer_count */
							KWP_reset_TimeOut();
							KWP_Set_TimeOut(l_P3MIN,P3MIN_WAIT);
							
							
						}
				
					}
				}

				else {

					/* If there is Link Init failure do nothing */
				}
	} 


		}
		
		
			

	return TRUE;
}


void uart_send_data(UARTContext *ctx, char data)
{
	gsize byte_written;
	GError *error = NULL;
	g_io_channel_write_chars(ctx->channel , &data, 1, &byte_written , &error);
	
	g_io_channel_flush(ctx->channel, &error);
	
	if(error)
	{
		g_print("UART Write Error");
	}
	else
	{
		g_print("sent data: %c",data);
	}

}

int kwp_init(void)
{

	const char *uart_dev = "/dev/ttyS4"; 
	int fd = open(uart_dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
	if (fd < 0) {
        	perror("UART open failed");
        	return 1;
	
	}

	// Configure UART (termios setup omitted for brevity)
	
	GIOChannel *channel = g_io_channel_unix_new(fd);
	g_io_channel_set_encoding(channel, NULL, NULL); // Binary mode
	g_io_channel_set_flags(channel, G_IO_FLAG_NONBLOCK, NULL);
	
	Global_uart_ctx = g_new(UARTContext ,1);
	Global_uart_ctx->channel = channel;
	
	g_io_add_watch(channel, G_IO_IN | G_IO_HUP | G_IO_ERR, KWP_uart_rx_callback, NULL);
	
	
	// Add periodic timeout (e.g., every 2 seconds)  
	g_timeout_add(1, KWP_Timer_Handler, NULL);

	return 0;
	
}

static uint8_t ISO9141_14230_GetChecksum(const uint8_t * p_Data, uint16_t p_Length, uint8_t p_TxRx)
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
void ISO9141_14230_Init(const ISO9141_14230_Init_S * p_ISO9141_14230Init_S)
{
	/* UART structure */

	/* Queue variables */

	
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
//	l_RefInitTime = 0;
	l_UARTIntrTxIndex = 0;
	l_P3MINTimeout = !FALSE;
//	l_MsgReceived = FALSE;
//	l_TxComp = FALSE;

	/* Bit code the Connect flags specific to ISO 9141 / 14230 */

	/* K Line only flag */
	if (CHECK_BITU32(p_ISO9141_14230Init_S->Flags,ISO9141_K_LINE_ONLY_BIT_POS) != FALSE) {

		/* Bit 0 */
		SET_BITU8(l_ConnectFlags, BIT_LINECONF);
	}

	/* Checksum flag:to ensure that the correct data is received and that the ECU is not damaged by corrupted messages. */
	if (CHECK_BITU32 (p_ISO9141_14230Init_S->Flags,ISO9141_NO_CHECKSUM_BIT_POS) != FALSE) {

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
	


	/* Interrupt handling */

	/* Configure GPIO  */

	GPIO_KWP_TXD.gpio_number = 36;
	GPIO_KWP_TXD.direction   = GPIO_DIRECTION_OUT;
	
	GPIO_KLINE_LLINE_SLCT.gpio_number = 44;
	GPIO_KLINE_LLINE_SLCT.direction = GPIO_DIRECTION_OUT;
	
	GPIO_KLINE_PULLUP_SLCT.gpio_number = 43;
	GPIO_KLINE_PULLUP_SLCT.direction = GPIO_DIRECTION_OUT;
	
	GPIO_LLINE_ISO_SLCT.gpio_number = 33;
	GPIO_LLINE_ISO_SLCT.direction = GPIO_DIRECTION_OUT;
	
	GPIO_KLINE_ISO_SLCT.gpio_number = 32;
	GPIO_KLINE_ISO_SLCT.direction = GPIO_DIRECTION_OUT;
	
	GPIO_KW_TX_5BAUD.gpio_number = 34;
	GPIO_KW_TX_5BAUD.direction = GPIO_DIRECTION_OUT;

	
	/* Configure GPIOs as O/P */

	T_gpio_PIN_config(&GPIO_KWP_TXD);
	T_gpio_PIN_config(&GPIO_KLINE_LLINE_SLCT);
	T_gpio_PIN_config(&GPIO_KLINE_PULLUP_SLCT);
	T_gpio_PIN_config(&GPIO_LLINE_ISO_SLCT);
	T_gpio_PIN_config(&GPIO_KLINE_ISO_SLCT);
	T_gpio_PIN_config(&GPIO_KW_TX_5BAUD);


	/* Update the Init Link status */
	l_InitStatus = NO_LINKINIT;

}

ISO9141_14230_RETCODE ISO9141_14230_Command(ISO9141_14230_Cmd_S * p_ISO9141_14230Cmd_SP)
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

				/* Clear temporary Tx and Rx variable */
				
				// UART commu

				/* Clear Intermediate Rx byte length ????? */
				l_RxLength = 0;

				/* Clear Tx length ????? */
				l_TxLength = 0;
				l_MsgReceived = FALSE;
				l_P3MINTimeout = !FALSE;
				l_TxComp = FALSE;

				/* Configure UART baud rate */

				//UART_Set_Baudrate(uartFd, baudrate);
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
		
	
				break;
		case DATA_BITS:
				/* Clear temporary Tx and Rx variable */
	
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
		l_ISO9141_14230LinkInit_S.InitType = p_ISO9141_14230Cmd_SP->IOCtlId;

		/* Update init data Length */
		l_ISO9141_14230LinkInit_S.Length = p_ISO9141_14230Cmd_SP->Length;

		/* Update the Init Data  */
		memcpy((void *)&l_ISO9141_14230LinkInit_S.Data, (uint8_t *) p_ISO9141_14230Cmd_SP->pData,sizeof(uint8_t) * l_ISO9141_14230LinkInit_S.Length);

		
		/* Update checksum if configured */
		if ((CHECK_BITU8(l_ConnectFlags, BIT_CHKSUM) == FALSE) && (l_ISO9141_14230LinkInit_S.InitType == FAST_INIT)) {

			/* Obtain the checksum for Init data */
			
			fl_ChkSum = ISO9141_14230_GetChecksum((void *) l_ISO9141_14230LinkInit_S.Data, l_ISO9141_14230LinkInit_S.Length, CHKSUM_TXDATA);

			/* Store the checksum as the last byte */
				l_ISO9141_14230LinkInit_S.Data  [l_ISO9141_14230LinkInit_S.Length] = fl_ChkSum;

			/* Update the length for the checksum */
			l_ISO9141_14230LinkInit_S.Length++;
		}
		

		/* Call the init function to perform the initialization of the  communication link */
		ISO9141_14230_LinkInit();	//To initialize communication link parameters for 5 Baud / Fast Init in Linux, we can use the stty command in the terminal.
		break;
	
	#
	case CLEAR_TX_BUFFER:	/* Clear Tx Buffers */

		/* Set Length to 0 */
		p_ISO9141_14230Cmd_SP->Length = 0;

		/* Clear all Tx Queues and Structures */
		memset((void *)&ISO9141_14230_TxMsg_S_Buffer, 0, sizeof(ISO9141_14230_TxMsg_S));

		
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


		break;
	case CLEAR_RX_BUFFER:	/* Clear Rx Buffers */
		/* Set Length to 0 */
		p_ISO9141_14230Cmd_SP->Length = 0;

		/* Clear Reception Queue */
		memset((void *)&ISO9141_14230_RxMsg_S_Buffer, 0, sizeof(ISO9141_14230_RxMsg_S));

		/* Clear intermediate frame bytes if any */
		l_UARTIntrTxIndex = 0;
		l_MsgReceived = FALSE;

		//l_P3MINTimeout = !FALSE;

		/* Clear Intermediate Rx byte length */
		l_RxLength = 0;

		/* Clear Tx length */
		l_TxLength = 0;

		break;
	
	default:
		/* Invalid command id */
		fl_RetCode = INVALID_COMMAND;
	}

	return fl_RetCode;
}

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

	
	/* Clear the RxLength */
	l_RxLength = 0;
	l_MsgReceived = FALSE;
	l_P3MINTimeout = FALSE;
	l_TxComp = FALSE;

	/* Clear the TxLength, Tx index and UART interrupt Tx index */
	l_TxLength = 0;
	l_TxIndex = 0;
	l_UARTIntrTxIndex = 0;



	/* Configure UART Tx as GPIO to generate the init pattern */
    	/* Config_Pin_Output(KW_TXD); */
	/*  Set_Pin_High(KW_TXD); */

	/* Disable RX Time OUT - if some Timeout caused in Idle Detection, The Init will Fail */
        /* UART_Change_RxTimeOut(UART_CH_ID_1, 0); */

	/* Update the reference Idle time - ?? Timer should have started ?? */
	/* ISO9141_14230_EnableBusMonitor(); */


	/* ISO9141_14230_RxMsg_S_Buffer initialize */
	memset((void*)&ISO9141_14230_RxMsg_S_Buffer,0,sizeof(ISO9141_14230_RxMsg_S_Buffer));
	
	/* Initialization status - Pending */
	l_InitStatus = LINKINIT_PENDING;
}

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
ISO9141_14230_RETCODE ISO9141_14230_WriteMsg(void)
{
	ISO9141_14230_RETCODE fl_RetCode = NO_ERROR;
	//ISO9141_14230_QSTATUS fl_TxQStatus;
	uint8_t fl_ChkSum;

	/* Calculate checksum if configured */
	if (CHECK_BITU8(l_ConnectFlags, BIT_CHKSUM) == FALSE) {

		/* Obtain the checksum for Tx data */
		fl_ChkSum = ISO9141_14230_GetChecksum(ISO9141_14230_TxMsg_S_Buffer.Data, ISO9141_14230_TxMsg_S_Buffer.Length, CHKSUM_TXDATA);

		/* Store the checksum as the last byte */
		ISO9141_14230_TxMsg_S_Buffer.Data[ISO9141_14230_TxMsg_S_Buffer.Length] = fl_ChkSum;

		/* Update the length for the checksum */
		ISO9141_14230_TxMsg_S_Buffer.Length++;
	}

	/* Add the message to be transmitted to the Queue */
	
	kwp_TxMsg_received = !FALSE;
	
	#if 0

	/* Restore the Tx frame if checksum is configured */
	if (CHECK_BITU8(l_ConnectFlags, BIT_CHKSUM) == FALSE) {

		/* Nullify the changes due to checksum */
		ISO9141_14230_TxMsg_S_Buffer.Length--;
		ISO9141_14230_TxMsg_S_Buffer.Data[ISO9141_14230_TxMsg_S_Buffer.Length] = 0;
	}
	#endif 
	/* Update Tx Timestamp */
	//get_time_stamp(timestamp_id[GARUDA_KWP_CH1], &p_ISO9141_14230TxMsg_SP->Timestamp);
	//get_data_logging_time_stamp(&p_ISO9141_14230TxMsg_SP->Timestamp);
	//  get_time_stamp(&p_ISO9141_14230TxMsg_SP->Timestamp);                               //here updating timestamp
	/* Check if the queue is Full */
#if 0	
	if (ISO9141_14230_Q_FULL == fl_TxQStatus) {

		/* Send Return Code */
		fl_RetCode = ISO9141_14230_TXQ_FULL;
	}
#endif

	/* Return code */
	return fl_RetCode;
}



void ISO9141_14230_TxTask(void)
{

	/* Check if transmission completed during non-init mdoe to remove the Tx frame from Queue */
	if ((l_InitStatus != LINKINIT_PENDING) && (l_TxComp != FALSE)) {

		/* If all the bytes are transmitted then remove the frame from Queue */
		/*ISO9141_14230_DelFromQ(ISO9141_14230_TX_Q, (ISO9141_14230_Q_S *) &fl_ISO9141_14230Tx_S);*/
		if (l_loopback) {
		
			ISO9141_14230_TxMsg_S_Buffer.Flags = 0x00000001;
			
			memset((void*)&ISO9141_14230_RxMsg_S_Buffer,0,sizeof(ISO9141_14230_RxMsg_S)) ;
			memcpy(&ISO9141_14230_TxMsg_S_Buffer,&ISO9141_14230_RxMsg_S_Buffer,sizeof(ISO9141_14230_RxMsg_S));
			
		
			/* check TXmsg struct buffer cleared */
			/* add Rxstruct buffer to Txmsg struct */
			/* clear Txmsg struct */

			/* Report error if RxQ full */
			/*if (ISO9141_14230_Q_FULL == fl_RxQStatus) {

				//Set Error Code and report to Call back function 
				App_ErrHandler(l_ProtocolId, RXQ_OVERFLOW);
			}*/
				
		}
		else{
			
		}
		
			/* Clear TxMsg buffer */		
			memset((void*)&ISO9141_14230_TxMsg_S_Buffer,0,sizeof(ISO9141_14230_TxMsg_S)) ;
			
			/* reset Txcompletion flag */
			l_TxComp = FALSE;
			kwp_TxMsg_received = FALSE;
	}

	/* Read a frame from Queue if(the initialization is not in the pending state AND(the minimum time between ECU response and 
	tester request has elapsed AND the previous frame is loaded to UART for transmission OR the Tx frame length is nonzero AND
	Index has not reached the Tx frame length) ) */
	if (((l_InitStatus != LINKINIT_PENDING) && (((l_P3MINTimeout != FALSE) && (l_TxLength == 0)) || ((l_TxLength != 0) && (l_TxIndex < l_TxLength)))) && (l_RxLength == 0))

	{
		if(kwp_TxMsg_received == !FALSE)
		{

			/* Initialize Index for Tx bytes when a new frame is to be transmitted */
			if ((l_TxLength == 0) && (l_TxIndex != 0)) {

				/* New frame - Restart Index */
				l_TxIndex = 0;
			}
			

			/* Load the Tx bytes onto UART */
			
			/* Save the Tx frame length and Load Tx Interbyte time for the new frame */
			if (l_TxIndex == 0) {

					/* Save Tx Length */
					l_TxLength = ISO9141_14230_TxMsg_S_Buffer.Length;

					/* Configure Tx Timeguard */
					//UART_Change_TimeGuard(UART_CH_ID_1, l_P4MIN);
					
					/* Start loading 1st byte Tx data onto the UART */
					uart_send_data(Global_uart_ctx,(char)ISO9141_14230_TxMsg_S_Buffer.Data[l_TxIndex]);
					l_TxIndex++;
					KWP_reset_TimeOut();
					KWP_Set_TimeOut(l_P4MIN,P4MIN_WAIT);
					
					
			}
			else
			{
				while (l_TxIndex < ISO9141_14230_TxMsg_S_Buffer.Length) {

						/* Start loading 2 nd byte Tx data onto the UART */
						if(l_p4min_timeout != FALSE)
						{
							/* Load to UART Tx */
							uart_send_data(Global_uart_ctx,(char)ISO9141_14230_TxMsg_S_Buffer.Data[l_TxIndex]);
							/* set P4MIN Time */
							KWP_reset_TimeOut();
							KWP_Set_TimeOut(l_P4MIN,P4MIN_WAIT);
							/* Increment Index */
							l_TxIndex++;
							l_p4min_timeout = FALSE;
						}
	
				}
					
					/* After send data reset TimeOut and P4MIN flag */
					KWP_reset_TimeOut();
					l_p4min_timeout = FALSE;
					
			}

		}
		else{
			/* Do nothing */
		}
	}
}

void ISO9141_14230_RxTask(void)
{
	uint8_t fl_ChkSum;
	uint16_t l_Chk78Length = 0, l_Chk78Frame = 0, l_Chk78Index = 0;
	bool l_Rxd78 = FALSE;
	
	ISO9141_14230_RxMsg_S_Buffer.Flags = 0;
	
	/* Poll the Length Queue only if not in Fast or Five Baud Initialization */
    	if(l_InitStatus != LINKINIT_PENDING)
	{
		/* checking RxMSg received flag */
		if(kwp_RxMsg_received != FALSE)
		{
			if(CHECK_BITU8(l_ConnectFlags, BIT_CHKSUM) == FALSE)
			{
				/* Obtain the checksum for Rx data */
				fl_ChkSum = ISO9141_14230_GetChecksum(ISO9141_14230_RxMsg_S_Buffer.Data,ISO9141_14230_RxMsg_S_Buffer.Length,CHKSUM_RXDATA);
				
				/* If Checksum is ok */
				if(fl_ChkSum == CHECKSUM_OK)
				{
					/* Update length */
					ISO9141_14230_RxMsg_S_Buffer.Length--;

					   /* Remove the checksum byte - Update only data*/
					ISO9141_14230_RxMsg_S_Buffer.Data[ISO9141_14230_RxMsg_S_Buffer.Length] = 0;

					   /* Update the Rx timestamp to the Rx frame */
			    //             ISO9141_14230_DelFromQ(ISO9141_14230_TIME_Q,(ISO9141_14230_Q_S *) &fl_ISO9141_14230Rx_S.Timestamp);
		#if 0
					   /* Check if the message qualifies for the filter */
					   if(J2534_checkFilter(&fl_ISO9141_14230Rx_S.Data[0], fl_ISO9141_14230Rx_S.Length,GARUDA_KWP_CH1) == J2534_PASS)
					   {
						   /* Store the message onto the Rx queue */
						   fl_RxQStatus = ISO9141_14230_AddToQ(ISO9141_14230_RX_Q
							   , (const ISO9141_14230_Q_S *)&fl_ISO9141_14230Rx_S,
							   0);
							   
							kwp_RxMsg_buffer_updated = true;

						   /* Report error if RxQ full */
						   if(ISO9141_14230_Q_FULL == fl_RxQStatus)
						   {
							   /* Set Error Code and report to Call back function */
							   App_ErrHandler(l_ProtocolId, RXQ_OVERFLOW);
						   }
					   }
					   else
					   {
					   
					   }
		#endif	
						/* after checkfilter implementation, this statement moves to if condition */
						kwp_RxMsg_buffer_updated = !FALSE;
					
				}
				/* If checksum not Ok, reported checksum error*/
				else
				{
					/* call error handler,reported checksum error */
					App_ErrHandler(l_ProtocolId, ECU_RESPCHKSUM_ERROR); 
					
					if(l_ProtocolId == KWP_PROTOCOL_ID)
					{
					   	/* Save the total length */
                       			     	l_Chk78Length = ISO9141_14230_RxMsg_S_Buffer.Length;
                       				l_Chk78Frame = 0;
					
					
						/* Check for the response $78 */
						
						/* doubt this for LOOP requied */
						for(l_Chk78Index = 0;l_Chk78Index  < l_Chk78Length; l_Chk78Index++)
						{
								/* Search if 7F and 78 are in the response */
								if( (ISO9141_14230_RxMsg_S_Buffer.Data[l_Chk78Index] == 0x7F) && (ISO9141_14230_RxMsg_S_Buffer.Data[l_Chk78Index+2] == 0x78))
								{
									/* Received 78 response */
									l_Rxd78 = !FALSE;
								   
								}
						}
						/* Check for positive response after 78 */
						if((l_Rxd78 != FALSE) && (l_Chk78Frame < l_Chk78Length))
						{
						   
						}
		                		/* If no 78 detected report error */
						if(l_Chk78Frame == 0)
						{
								
						}

					}
					else{
						 /* Call application error handler */
					}
					
				}
			}
			/* If no checksum configured */
			else
			{
				/* Update the Rx timestamp to the Rx frame */
			//	ISO9141_14230_DelFromQ(ISO9141_14230_TIME_Q,(ISO9141_14230_Q_S *) &fl_ISO9141_14230Rx_S.Timestamp);
	#if 0
		       			/* Check if the message qualifies for the filter */
			       if(J2534_checkFilter(&fl_ISO9141_14230Rx_S.Data[0],fl_ISO9141_14230Rx_S.Length,GARUDA_KWP_CH1) == J2534_PASS)
			       {
				   /* Store the message onto the Rx queue */
				//   fl_RxQStatus = ISO9141_14230_AddToQ(ISO9141_14230_RX_Q ,(const ISO9141_14230_Q_S *)&fl_ISO9141_14230Rx_S,0);
							
				 	kwp_RxMsg_buffer_updated = true;
							
				   /* Report error if RxQ full */
				   if(ISO9141_14230_Q_FULL == fl_RxQStatus)
				   {
				       /* Set Error Code and report to Call back function */
				       App_ErrHandler(l_ProtocolId, RXQ_OVERFLOW);
				   }
			       }
				else
				{
					   
				}
	#endif		
				
				kwp_RxMsg_buffer_updated = !FALSE;
			}
			
		}
		else 
		{
			/* do nothing */
		}	
	}
	else
	{
		/*do nothing */
	}

}

/* First Byte received */
void App_FirstByteRxd(uint32_t l_ProtocolId, uint32_t p_timestamp)
{
        uint8_t usb_tx[512] = {0};

        //usb_tx[0] = l_ProtocolId;
        usb_tx[0] = (l_ProtocolId >> 0) & 0xFF;
    	usb_tx[1] = (l_ProtocolId >> 8) & 0xFF;
    	usb_tx[2] = (l_ProtocolId >> 16) & 0xFF;
    	usb_tx[3] = (l_ProtocolId >> 24) & 0xFF;
    
        
        usb_tx[4] = 0x0A;
        usb_tx[5] = 0;
        usb_tx[6] = 0;
        usb_tx[7] = 0;
        usb_tx[8] = 0;
        usb_tx[9] = 2;
        usb_tx[10] = 0;
        usb_tx[11] = 0;
        usb_tx[12] = 0;
        usb_tx[13] = (uint8_t)(p_timestamp & 0xFF);
        usb_tx[14] = (uint8_t)((p_timestamp >> 8) & 0xFF);
        usb_tx[15] = (uint8_t)((p_timestamp >> 16) & 0xFF);
        usb_tx[16] = (uint8_t)((p_timestamp >> 24) & 0xFF);
        usb_tx[17] = 0;
        
        (void)host_write((void *)usb_tx, 18);
        
}

void PassThruReadMsgResp_KWP (void)
{
    static uint16_t fl_KWPRX_LocalLen;
    static uint8_t fl_KWPRX_SegNo=0;
   // ISO9141_14230_RETCODE fl_ISO9141_14230RetStatus;
    uint8_t fl_USB_tx_data_U8A[512] ;
    uint16_t fl_IdxLen;
    uint32_t current_time_stamp = 0;

    if(0 == fl_KWPRX_SegNo)
    {
        /* Call the Read Message function */
        /*fl_ISO9141_14230RetStatus = ISO9141_14230_ReadMsg(&l_App_ISO9141_14230RxMsg_S);*/

        /* Update the Error Code based on the return */
        if(kwp_RxMsg_buffer_updated != FALSE)
        {
            /* Determine the Response */
            //fl_USB_tx_data_U8A[0] = get_ISO9141_or_14230();//HFCP.c
            fl_USB_tx_data_U8A[0]= (l_ProtocolId >> 0) & 0xFF;
            fl_USB_tx_data_U8A[1]= (l_ProtocolId >> 8) & 0xFF;
            fl_USB_tx_data_U8A[2]= (l_ProtocolId >> 16) & 0xFF;
            fl_USB_tx_data_U8A[3]= (l_ProtocolId >> 24) & 0xFF;
            fl_USB_tx_data_U8A[4] = KWP_Receive_msg;
            fl_USB_tx_data_U8A[5] = fl_KWPRX_SegNo;
            fl_USB_tx_data_U8A[6] = STATUS_NOERROR;

            /* Store the read message */
            fl_USB_tx_data_U8A[7] = (uint8_t)((ISO9141_14230_RxMsg_S_Buffer.Length )      & 0xFF);
            fl_USB_tx_data_U8A[8] = (uint8_t)((ISO9141_14230_RxMsg_S_Buffer.Length >> 8)  & 0xFF);

            fl_USB_tx_data_U8A[9] = (uint8_t)((ISO9141_14230_RxMsg_S_Buffer.Flags )      & 0xFF);
            fl_USB_tx_data_U8A[10] = (uint8_t)((ISO9141_14230_RxMsg_S_Buffer.Flags >> 8)  & 0xFF);
            fl_USB_tx_data_U8A[11] = (uint8_t)((ISO9141_14230_RxMsg_S_Buffer.Flags >> 16) & 0xFF);
            fl_USB_tx_data_U8A[12] = (uint8_t)((ISO9141_14230_RxMsg_S_Buffer.Flags >> 24) & 0xFF);

            current_time_stamp = ISO9141_14230_RxMsg_S_Buffer.Timestamp;
            fl_USB_tx_data_U8A[13] = (uint8_t)((current_time_stamp )      & 0xFF);
            fl_USB_tx_data_U8A[14] = (uint8_t)((current_time_stamp >> 8)  & 0xFF);
            fl_USB_tx_data_U8A[15] = (uint8_t)((current_time_stamp >> 16) & 0xFF);
            fl_USB_tx_data_U8A[16] = (uint8_t)((current_time_stamp >> 24) & 0xFF);
            /* Make the Local Length as 0 As it first frame*/
            fl_KWPRX_LocalLen = 0;
            /* 50 bytes of Space in this USB Frame */
            
            /* size needs to check */
            for(fl_IdxLen = 0;fl_IdxLen < ISO9141_14230_RxMsg_S_Buffer.Length;fl_IdxLen++)
            {
                fl_USB_tx_data_U8A[17+fl_IdxLen]  =  ISO9141_14230_RxMsg_S_Buffer.Data[fl_IdxLen];
                fl_KWPRX_LocalLen++;
            }
            #if 0
            if(ISO9141_14230_RxMsg_S_Buffer.Length > 50)
            {
                /* Its a Segmented Message Transfer Set Flag */
                l_KWPRX_SegTrnsfr = 1;
                /* Change the Segment Field on USB Frame */
                fl_KWPRX_SegNo = 1;
                fl_USB_tx_data_U8A[2] = fl_KWPRX_SegNo;
            }
            #endif
           // else
           // {
            	/* Reset RxMsg buffer variable */
                kwp_RxMsg_received = FALSE;
		kwp_RxMsg_buffer_updated = FALSE;
		
		/* Reset RxMsg struct */
		memset((void*)&ISO9141_14230_RxMsg_S_Buffer,0,sizeof(ISO9141_14230_RxMsg_S_Buffer)) ;
           // }
           
            (void)host_write((void *)fl_USB_tx_data_U8A, 512);
       
        }
    }
 
    else
    {
        /* Do Nothing */
    }
}



void ISO9141_14230_Reset(void)
{
	/* Reset UART communication */

	memset((void *)&ISO9141_14230_TxMsg_S_Buffer, 0, sizeof(ISO9141_14230_TxMsg_S_Buffer));
	memset((void *)&ISO9141_14230_RxMsg_S_Buffer, 0, sizeof(ISO9141_14230_RxMsg_S_Buffer));
	
	/* Initializuint8_te Link Init structure */
	memset((void *)&l_ISO9141_14230LinkInit_S, 0, sizeof(ISO9141_14230_LinkInit_S));

	/* Initialize Init Return data structure */
	memset((void *)&l_ISO9141_14230LinkInitRet_S, 0, sizeof(ISO9141_14230_LinkInitRet_S));


	timer_init_flag = FALSE;
	fastinit_start_comm_req_start_flag = FALSE;
	fastinit_start_comm_req_end_flag = FALSE;
	kwp_RxMsg_received = FALSE;
	kwp_RxMsg_buffer_updated = FALSE;
	kwp_TxMsg_received = FALSE;

	l_p4min_timeout = false;

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
	//l_RefIdleTime = 0;
	//l_RefInitTime = 0;
	l_UARTIntrTxIndex = 0;
	l_P3MINTimeout = !FALSE;
	l_MsgReceived = FALSE;
	l_TxComp = FALSE;

	/*DisConnect K & L from OBD Connector */
//    Set_Pin_Low(LLINE_ISO_SLCT);
	//   Set_Pin_Low(KLINE_ISO_SLCT);
}

void App_InitData(ISO9141_14230_LinkInitRet_S *p_InitData_SP)
{

    uint16_t loop_count;
    uint8_t usb_tx[512] = {0};
    
    uint32_t fl_ProtocolId = p_InitData_SP->ProtocolId;

    /* Prepare the USB transmit frame */
   
  
    /*memcpy(&usb_tx[0], &protocol_id, sizeof(int));*/
    
    usb_tx[0] = (fl_ProtocolId >> 0) & 0xFF;
    usb_tx[1] = (fl_ProtocolId >> 8) & 0xFF;
    usb_tx[2] = (fl_ProtocolId >> 16) & 0xFF;
    usb_tx[3] = (fl_ProtocolId >> 24) & 0xFF;
    
    usb_tx[4] = IOCTL_COMMAND;
    usb_tx[5] = p_InitData_SP->IOCtlId;
    usb_tx[6] = STATUS_NOERROR;
    usb_tx[7] = (p_InitData_SP->Length) & 0xFF;
    usb_tx[8] = (p_InitData_SP->Length >> 8) & 0xFF;
    for(loop_count=0; loop_count < p_InitData_SP->Length; loop_count++)
    {
        usb_tx[9+loop_count] = p_InitData_SP->Data[loop_count];
    }
   // (void)Garuda_Tx_data_on_USB(&usb_tx[0],64,DONT_RELEASE_ISR/*DONT_RELEASE*/);//made changes in HFCP.c
   (void)host_write((void *)usb_tx, 512);
}

/* Error Handler */
//uint8_t apperrhandler;

void App_ErrHandler(uint32_t protocol_id, uint8_t error_code)
{

  uint8_t usb_tx[512] = {0};

    /* Prepare the USB transmit frame */
    //usb_tx[0] = protocol_id;
    
    /*memcpy(&usb_tx[0], &protocol_id, sizeof(int));*/
    usb_tx[0] = (protocol_id >> 0) & 0xFF;
    usb_tx[1] = (protocol_id >> 8) & 0xFF;
    usb_tx[2] = (protocol_id >> 16) & 0xFF;
    usb_tx[3] = (protocol_id >> 24) & 0xFF;

    //apperrhandler = error_code;
    
     /* Report Init error */
    switch(error_code) {
    	case PATTERN_BYTE_TIMEOUT:
    	case KEY_BYTE1_TIMEOUT:
    	case KEY_BYTE2_TIMEOUT:
    	case ADDRESS_BYTE_TIMEOUT:
    		usb_tx[4] = IOCTL_COMMAND;
      		usb_tx[5] = FIVE_BAUD_INIT;
       		usb_tx[6] = ERR_FAILED; 
       		(void)host_write((void *)usb_tx, 7);
       		break;
       	case FASTINIT_RESP_TIMEOUT:
       	case FASTINIT_RESPCHKSUM_ERROR:
       		usb_tx[4] = IOCTL_COMMAND;
      		usb_tx[5] = FAST_INIT;
       		usb_tx[6] = ERR_FAILED; 
       		(void)host_write((void *)usb_tx, 7);
       		break;
       	case MSG_CORRUPTED:
       		usb_tx[4] = SND_MSG_CORRUPTED;
      		usb_tx[5] = MSG_CORRUPTED;
       		usb_tx[6] = ERR_FAILED; 
       		(void)host_write((void *)usb_tx, 7);
       		break;
       	case ECU_RESP_TIMEOUT:
       		usb_tx[4] = SND_ECU_RESP;
      		usb_tx[5] = ECU_RESP_TIMEOUT;
       		usb_tx[6] = ERR_FAILED; 
       		(void)host_write((void *)usb_tx, 7);
       		break;
       	case ECU_RESPCHKSUM_ERROR:
       		usb_tx[4] = SND_ECU_RESPCHKSUM;
      		usb_tx[5] = ECU_RESPCHKSUM_ERROR;
       		usb_tx[6] = ERR_FAILED; 
       		(void)host_write((void *)usb_tx, 7);
       		break;	
       	default:
       		break;
  		
    }
   
}





