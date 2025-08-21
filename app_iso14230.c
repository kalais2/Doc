/**
 *  @file app_iso14230.c
 *
 *  This module implements the application for ISO 9141 / 14230 protocol
 *
 *  Copyright (c) 2023, Capgemini - Intelligent Devices
 */

/******************************************************************************
*                    I N C L U D E   F I L E S
*******************************************************************************/
#include "app_iso14230.h"
#include "j2534_filter.h"
#include "hfcp.h"

/******************************************************************************
*               I N T E R N A L   D E F I N I T I O N S
*******************************************************************************/


/******************************************************************************
*         P R I V A T E    F U N C T I O N    P R O T O T Y P E S
*******************************************************************************/
/******************************************************************************
*                   P R I V A T E   V A R I A B L E S
*******************************************************************************/
static uint8_t l_KWPRX_SegTrnsfr=0;
static ISO9141_14230_RxMsg_S l_App_ISO9141_14230RxMsg_S;

/******************************************************************************
*                   E X P O R T E D   F U N C T I O N S
*******************************************************************************/
/******************************************************************************
*                   P R I V A T E   F U N C T I O N S
*******************************************************************************/

/* Determine the Pullup for K & L line based on battery voltage */
uint8_t CheckIf_VBATTis24V(void)
{
    uint32_t Batt_Vg;
  //  while(FALSE == Read_BatteryVoltage(&Batt_Vg));//ADC_Mid.c
    
    if(Batt_Vg <= 15000)
    {
        return FALSE;
    }
    else
    {
        return TRUE;
    }
}

/* Application return data */
void App_InitData(ISO9141_14230_LinkInitRet_S *p_InitData_SP)
{

    uint8_t loop_count;
    uint8_t usb_tx[64] = {0};

    /* Prepare the USB transmit frame */
    usb_tx[0] = p_InitData_SP->ProtocolId;
    usb_tx[1] = IOCTL_COMMAND;
    usb_tx[2] = p_InitData_SP->IOCtlId;
    usb_tx[3] = STATUS_NOERROR;
    usb_tx[4] = p_InitData_SP->Length;
    for(loop_count=0;
        loop_count < p_InitData_SP->Length;
        loop_count++)
    {
        usb_tx[5+loop_count] = p_InitData_SP->Data[loop_count];
    }
   // (void)Garuda_Tx_data_on_USB(&usb_tx[0],64,DONT_RELEASE_ISR/*DONT_RELEASE*/);//made changes in HFCP.c
}

/* Error Handler */
uint8_t apperrhandler;

void App_ErrHandler(uint8_t protocol_id, uint8_t error_code)
{

  uint8_t usb_tx[64] = {0};

    /* Prepare the USB transmit frame */
    usb_tx[0] = protocol_id;

    apperrhandler = error_code;

    /* Report Init error */
    if((error_code == FASTINIT_RESP_TIMEOUT) ||
       (error_code == FASTINIT_RESPCHKSUM_ERROR))
    {
        usb_tx[1] = IOCTL_COMMAND;
        usb_tx[2] = FAST_INIT;
        usb_tx[3] = ERR_FAILED;
 //       (void)Garuda_Tx_data_on_USB(&usb_tx[0],64,DONT_RELEASE_ISR/*DONT_RELEASE*/);//made changes in HFCP.c
    }
    else if((error_code == PATTERN_BYTE_TIMEOUT) ||
            (error_code == KEY_BYTE1_TIMEOUT) ||
            (error_code == KEY_BYTE2_TIMEOUT) ||
            (error_code == ADDRESS_BYTE_TIMEOUT))
    {
        usb_tx[1] = IOCTL_COMMAND;
        usb_tx[2] = FIVE_BAUD_INIT;
        usb_tx[3] = ERR_FAILED;
//        (void)Garuda_Tx_data_on_USB(&usb_tx[0],64,DONT_RELEASE_ISR/*DONT_RELEASE*/);//made changes in HFCP.c
    }
    else
    {

    }
}

/* First Byte received */
void App_FirstByteRxd(uint8_t l_ProtocolId, uint32_t p_timestamp)
{
        uint8_t usb_tx[64] = {0};

        usb_tx[0] = l_ProtocolId;
        usb_tx[1] = 0x0A;
        usb_tx[2] = 0;
        usb_tx[3] = 0;
        usb_tx[4] = 0;
        usb_tx[5] = 0;
        usb_tx[6] = 2;
        usb_tx[7] = 0;
        usb_tx[8] = 0;
        usb_tx[9] = 0;
        usb_tx[10] = (uint8_t)(p_timestamp & 0xFF);
        usb_tx[11] = (uint8_t)((p_timestamp >> 8) & 0xFF);
        usb_tx[12] = (uint8_t)((p_timestamp >> 16) & 0xFF);
        usb_tx[13] = (uint8_t)((p_timestamp >> 24) & 0xFF);
        usb_tx[14] = 0;
//        (void)Garuda_Tx_data_on_USB(&usb_tx[0],64,DONT_RELEASE_ISR/*DONT_RELEASE*/);//made changes in HFCP.c
}

void PassThruReadMsgResp_KWP (void)
{
    static uint16_t fl_KWPRX_LocalLen;
    static uint8_t fl_KWPRX_SegNo=0;
    ISO9141_14230_RETCODE fl_ISO9141_14230RetStatus;
    uint8_t fl_USB_tx_data_U8A[64] ;
    uint16_t fl_IdxLen;
    uint32_t current_time_stamp = 0;

    if((0 == l_KWPRX_SegTrnsfr)&&(0 == fl_KWPRX_SegNo))
    {
        /* Call the Read Message function */
        fl_ISO9141_14230RetStatus =
            ISO9141_14230_ReadMsg(&l_App_ISO9141_14230RxMsg_S);

        /* Update the Error Code based on the return */
        if(fl_ISO9141_14230RetStatus != ISO9141_14230_RXQ_EMPTY)
        {
            /* Determine the Response */
//            fl_USB_tx_data_U8A[0] = get_ISO9141_or_14230();//HFCP.c
            fl_USB_tx_data_U8A[1] = KWP_Receive_msg;
            fl_USB_tx_data_U8A[2] = fl_KWPRX_SegNo;
            fl_USB_tx_data_U8A[3] = STATUS_NOERROR;

            /* Store the read message */
            fl_USB_tx_data_U8A[4] = (uint8_t)((l_App_ISO9141_14230RxMsg_S.Length )      & 0xFF);
            fl_USB_tx_data_U8A[5] = (uint8_t)((l_App_ISO9141_14230RxMsg_S.Length >> 8)  & 0xFF);

            fl_USB_tx_data_U8A[6] = (uint8_t)((l_App_ISO9141_14230RxMsg_S.Flags )      & 0xFF);
            fl_USB_tx_data_U8A[7] = (uint8_t)((l_App_ISO9141_14230RxMsg_S.Flags >> 8)  & 0xFF);
            fl_USB_tx_data_U8A[8] = (uint8_t)((l_App_ISO9141_14230RxMsg_S.Flags >> 16) & 0xFF);
            fl_USB_tx_data_U8A[9] = (uint8_t)((l_App_ISO9141_14230RxMsg_S.Flags >> 24) & 0xFF);

            current_time_stamp = l_App_ISO9141_14230RxMsg_S.Timestamp;
            fl_USB_tx_data_U8A[10] = (uint8_t)((current_time_stamp )      & 0xFF);
            fl_USB_tx_data_U8A[11] = (uint8_t)((current_time_stamp >> 8)  & 0xFF);
            fl_USB_tx_data_U8A[12] = (uint8_t)((current_time_stamp >> 16) & 0xFF);
            fl_USB_tx_data_U8A[13] = (uint8_t)((current_time_stamp >> 24) & 0xFF);
            /* Make the Local Length as 0 As it first frame*/
            fl_KWPRX_LocalLen = 0;
            /* 50 bytes of Space in this USB Frame */
            for(fl_IdxLen = 0;
                fl_IdxLen < 50;
                fl_IdxLen++)
            {
                fl_USB_tx_data_U8A[14+fl_IdxLen]  =  l_App_ISO9141_14230RxMsg_S.Data[fl_IdxLen];
                fl_KWPRX_LocalLen++;
            }
            if(l_App_ISO9141_14230RxMsg_S.Length > 50)
            {
                /* Its a Segmented Message Transfer Set Flag */
                l_KWPRX_SegTrnsfr = 1;
                /* Change the Segment Field on USB Frame */
                fl_KWPRX_SegNo = 1;
                fl_USB_tx_data_U8A[2] = fl_KWPRX_SegNo;
            }
//            (void)Garuda_Tx_data_on_USB(&fl_USB_tx_data_U8A[0],64,DONT_RELEASE_ISR/*DONT_RELEASE*/);//made changes in HFCP.c
        }
    }
    /* If Segmented Transfer Send Next Segments */
    else if ((1 == l_KWPRX_SegTrnsfr) && (0 != fl_KWPRX_SegNo))
    {
        /* Next Segment */
        fl_KWPRX_SegNo++;
        /* Determine the Response */
//        fl_USB_tx_data_U8A[0] = get_ISO9141_or_14230();//HFCP.c
        fl_USB_tx_data_U8A[1] = KWP_Receive_msg;
        fl_USB_tx_data_U8A[2] = fl_KWPRX_SegNo;
        /* 61 bytes of Space in this USB Frame */
        for(fl_IdxLen = 0;
            fl_IdxLen < 61;
            fl_IdxLen++)
        {
            fl_USB_tx_data_U8A[3+fl_IdxLen]  =  l_App_ISO9141_14230RxMsg_S.Data[fl_KWPRX_LocalLen];
            fl_KWPRX_LocalLen++;
            /* If All Data Copied then Stop Coping and MArk End of Segmented Transfer */
            if(fl_KWPRX_LocalLen >= l_App_ISO9141_14230RxMsg_S.Length)
            {
                fl_KWPRX_SegNo = 0;
                l_KWPRX_SegTrnsfr = 0;
                break;
            }
        }
 //       (void)Garuda_Tx_data_on_USB(&fl_USB_tx_data_U8A[0],64,DONT_RELEASE_ISR/*DONT_RELEASE*/);//made changes in HFCP.c
    }
    else
    {
        /* Do Nothing */
    }
}

//void KWPTask(void *PvParam);

// Aboce line replaced below..............................

void* KWPTask(void *PvParam);
//extern  UART_InitTypeDef UART_midConfig_v[nUART] ;		// commented for uart dependency required..

#if 0
void KWPFreeRTOS_Init(void)
{
 //:   Timer_Init();
    
 /*  if(pdTRUE != xTaskCreate(KWPTask, "KWP_Task",KWP_TASK_STACK_SIZE, 
                           (void *) NULL, KWP_TASK_PRIORITY, ( TaskHandle_t * )NULL))
  {
    
  }*/
  
  // Above if condition replaced below................................
  
  pthread_t kwp_task;
  pthread_attr_t thread_attr;
  int stack_size = 1024;                                         
  struct sched_param sched_params;
  
  // Set stack size							                                
  pthread_attr_setstacksize(&thread_attr, stack_size);
  
  // Set thread priority						                            // need to modify according to linux. priority.
  sched_params.sched_priority = 10;
  pthread_attr_setschedparam(&thread_attr, &sched_params);
  
  if(pthread_create(&kwp_task, &thread_attr, KWPTask, NULL) !=0)
  {

  }
        
}
#endif

/*void KWPTask(void *PvParam)
{
    for(;;)
    {
        if(get_KWP_status())
        {
            ISO9141_14230_RxTask();
            PassThruReadMsgResp_KWP();
            ISO9141_14230_TxTask();
        }
        vTaskDelay(2);
    }
}*/

// Above KWPTask function replaced below...............................

void* KWPTask(void *PvParam)
{
#if 0
    for(;;)
    {
//        if(get_KWP_status())//HFCP.c
        {
            ISO9141_14230_RxTask();
            PassThruReadMsgResp_KWP();
            ISO9141_14230_TxTask();
        }
        //vTaskDelay(2);
        
        // Above vTaskDelay function replaced below...................
        struct timespec delay;
    	delay.tv_sec = 2;
    	nanosleep(&delay, NULL);
    }
#endif
}

#if DELETED_CODE
  //if(get_KWP_status())
  //{
      //ISO9141_14230_RxTask();
      //PassThruReadMsgResp_KWP();
      //ISO9141_14230_TxTask();
  //}
  //for(;;)
  //{
    //UART_midConfig_v[UART_CH_ID_1].baudrate = 19200;
    //UART_midConfig_v[UART_CH_ID_1].rxQueueSz = 100;
    //UART_midConfig_v[UART_CH_ID_1].txQueueSz = 100;
    //UART_MidInit(UART_CH_ID_1, UART_midConfig_v[UART_CH_ID_1]);
    //uint8_t count = 10;
//
    //while(count--)
    //{
        //uartMidSend (UART_CH_ID_1,&count, 1);
    //}
//
    //UART_MidDisable(UART_CH_ID_1);
   //
    /////* Configure UART Tx as GPIO to generate the init pattern */
    //Config_Pin_Output(KW_TXD);
    //Set_Pin_High(KW_TXD);
    //for(int i=0; i< 50000000; i++);
    //for(int i=0; i< 50000000; i++);
////
    //Set_Pin_Low(KW_TXD);
    ////Set_Pin_High(KW_TXD);
    //for(int i=0; i< 50000000; i++);
    //for(int i=0; i< 50000000; i++);
    ////Set_Pin_Low(KW_TXD);
//
    //Config_Pin_UART_Mode(KW_TXD);
    //UART_MidEnable(UART_CH_ID_1);
    //
    //UART_midConfig_v[UART_CH_ID_1].baudrate = 19200;
    //UART_midConfig_v[UART_CH_ID_1].rxQueueSz = 100;
    //UART_midConfig_v[UART_CH_ID_1].txQueueSz = 100;
    ////UART_MidInit(UART_CH_ID_1, UART_midConfig_v[UART_CH_ID_1]);
     //count = 20;
//
    //while(count--)
    //{
        //uartMidSend (UART_CH_ID_1,&count, 1);
    //}
//
///*
    //Config_Pin_Output(KW_TXD);
    //Set_Pin_High(KW_TXD);
    //for(int i=0; i< 50000000; i++);
    //for(int i=0; i< 50000000; i++);
//
    //Set_Pin_Low(KW_TXD);
    //Set_Pin_High(KW_TXD);
    //for(int i=0; i< 50000000; i++);
    //for(int i=0; i< 50000000; i++);
    //Set_Pin_Low(KW_TXD);
//*/
        //UART_midConfig_v[UART_CH_ID_1].baudrate = 19200;
    //UART_midConfig_v[UART_CH_ID_1].rxQueueSz = 100;
    //UART_midConfig_v[UART_CH_ID_1].txQueueSz = 100;
    //UART_MidInit(UART_CH_ID_1, UART_midConfig_v[UART_CH_ID_1]);
     //count = 30;
//
    //while(count--)
    //{
        //uartMidSend (UART_CH_ID_1,&count, 1);
    //}
    //vTaskDelay(5000 );
//
    //} 
#endif 

/******************************************************************************
 * R E V I S I O N   H I S T O R Y
 * $History: $
 * Version  Author  Date
 * 1.0  Karthik Subramanian    June 06, 2008
 * 1.1  Sanjeeva & Mahadeva    March 06, 2014
 * 1.2  Mahadeva               April 21, 2014
********************************************************************************
 * 1.0  Initial Version
*******************************************************************************/
/*******************************************************************************
 * 1.0  The Garuda_Tx_data_on_USB Function is Replaced For Garuda
*******************************************************************************/
/*******************************************************************************
 * 1.1  KWP Rx/Tx Task and Read Msg Responce handling  Moved to 2ms Scheduler
instead running it from timer 0 Interrupt
*******************************************************************************/
/*******************************************************************************
 * 1.2  Get_KLPullUp renamed as CheckIf_VBATTis24V. VBATT Read is performed
to read the Battery Voltage(Instead of Hardcoded value)
*******************************************************************************/

