/* Includes ------------------------------------------------------------------*/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include <stdint.h>
#include <string.h>
#include "spirit1_appli.h"
#include "MCU_Interface.h" 
#include "SPIRIT1_Util.h"
#include "esp_log.h"

//TODO
#define HAL_Delay(x) vTaskDelay(pdMS_TO_TICKS(x))

/** @addtogroup USER
* @{
*/

/** @defgroup SPIRIT1_APPLI
* @brief User file to configure spirit1 transceiver for desired frequency and 
* @feature.
* @{
*/

/* Private typedef -----------------------------------------------------------*/

/**
* @brief RadioDriver_t structure fitting
*/
RadioDriver_t spirit_cb =
{
  .Init = Spirit1InterfaceInit, 
  .GpioIrq = Spirit1GpioIrqInit,
  .RadioInit = Spirit1RadioInit,
  .SetRadioPower = Spirit1SetPower,
  .PacketConfig = Spirit1PacketConfig,
  .SetPayloadLen = Spirit1SetPayloadlength,
  .SetDestinationAddress = Spirit1SetDestinationAddress,
  .EnableTxIrq = Spirit1EnableTxIrq,
  .EnableRxIrq = Spirit1EnableRxIrq,
  .DisableIrq = Spirit1DisableIrq,
  .SetRxTimeout = Spirit1SetRxTimeout,
  .EnableSQI = Spirit1EnableSQI,
  .SetRssiThreshold = Spirit1SetRssiTH,
  .ClearIrqStatus = Spirit1ClearIRQ,
  .StartRx = Spirit1StartRx,
  .StartTx = Spirit1StartTx,
  .GetRxPacket = Spirit1GetRxPacket
};

/**
* @brief MCULowPowerMode_t structure fitting
*/
MCULowPowerMode_t MCU_LPM_cb =
{
  .McuStopMode = MCU_Enter_StopMode,
  .McuStandbyMode = MCU_Enter_StandbyMode,
  .McuSleepMode = MCU_Enter_SleepMode
}; 

/**
* @brief RadioLowPowerMode_t structure fitting
*/
RadioLowPowerMode_t Radio_LPM_cb =
{
  .RadioShutDown = RadioPowerOFF,
  .RadioStandBy = RadioStandBy,
  .RadioSleep = RadioSleep,
  .RadioPowerON = RadioPowerON
};

/**
* @brief GPIO structure fitting
*/
SGpioInit xGpioIRQ={
  SPIRIT_GPIO_IRQ,
  SPIRIT_GPIO_MODE_DIGITAL_OUTPUT_LP,
  SPIRIT_GPIO_DIG_OUT_IRQ
};

/**
* @brief Radio structure fitting
*/
SRadioInit xRadioInit = {
  XTAL_OFFSET_PPM,
  BASE_FREQUENCY,
  CHANNEL_SPACE,
  CHANNEL_NUMBER,
  MODULATION_SELECT,
  DATARATE,
  FREQ_DEVIATION,
  BANDWIDTH
};




/**
* @brief Packet Basic structure fitting
*/
PktBasicInit xBasicInit={
  PREAMBLE_LENGTH,
  SYNC_LENGTH,
  SYNC_WORD,
  LENGTH_TYPE,
  LENGTH_WIDTH,
  CRC_MODE,
  CONTROL_LENGTH,
  EN_ADDRESS,
  EN_FEC,
  EN_WHITENING
};


/**
* @brief Address structure fitting
*/
PktBasicAddressesInit xAddressInit={
  EN_FILT_MY_ADDRESS,
  MY_ADDRESS,
  EN_FILT_MULTICAST_ADDRESS,
  MULTICAST_ADDRESS,
  EN_FILT_BROADCAST_ADDRESS,
  BROADCAST_ADDRESS
};


#ifdef CSMA_ENABLE
  /**
  * @brief CSMA structure fitting
  */
  CsmaInit xCsmaInit={
    PERSISTENT_MODE_EN, /* CCA may optionally be persistent, i.e., rather than 
    entering backoff when the channel is found busy, CCA continues until the 
    channel becomes idle or until the MCU stops it.
    The thinking behind using this option is to give the MCU the possibility of 
    managing the CCA by itself, for instance, with the allocation of a 
    transmission timer: this timer would start when MCU finishes sending out 
    data to be transmitted, and would end when MCU expects that its transmission
    takes place, which would occur after a period of CCA.
    The choice of making CCA persistent should come from trading off 
    transmission latency,under the direct control of the MCU, and power 
    consumption, which would be greater due to a busy wait in reception mode.
                          */
    CS_PERIOD,
    CS_TIMEOUT,
    MAX_NB,
    BU_COUNTER_SEED,
    CU_PRESCALER
  };
#endif 
  
/* Private define ------------------------------------------------------------*/
#define TIME_UP                                         0x01

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
RadioDriver_t *pRadioDriver;
MCULowPowerMode_t *pMCU_LPM_Comm;
RadioLowPowerMode_t  *pRadio_LPM_Comm;
/*Flags declarations*/
volatile FlagStatus xRxDoneFlag = RESET, xTxDoneFlag=RESET, cmdFlag=RESET;
volatile FlagStatus xStartRx=RESET, rx_timeout=RESET, exitTime=RESET;
volatile FlagStatus PushButtonStatusWakeup=RESET;
volatile FlagStatus PushButtonStatusData=RESET;
/*IRQ status struct declaration*/
SpiritIrqs xIrqStatus;
//__IO uint32_t KEYStatusData = 0x00;
static AppliFrame_t xTxFrame, xRxFrame;
uint8_t DestinationAddr;
uint8_t TxFrameBuff[MAX_BUFFER_LEN] = {0x00};
uint16_t exitCounter = 0;
uint16_t txCounter = 0;
uint16_t wakeupCounter = 0;
uint16_t dataSendCounter = 0x00;

/* Private function prototypes -----------------------------------------------*/

//void HAL_Spirit1_Init(void);
//void Enter_LP_mode(void);
//void Exit_LP_mode(void);
//void MCU_Enter_StopMode(void);
//void MCU_Enter_StandbyMode(void);
//void MCU_Enter_SleepMode(void);
//void RadioPowerON(void);
//void RadioPowerOFF(void);
//void RadioStandBy(void);
//void RadioSleep(void);
//void AppliSendBuff(AppliFrame_t *xTxFrame, uint8_t cTxlen);
//void AppliReceiveBuff(uint8_t *RxFrameBuff, uint8_t cRxlen);
//void P2P_Init(void);
//void STackProtocolInit(void);
//void BasicProtocolInit(void);
//void P2PInterruptHandler(void);
//void Set_KeyStatus(FlagStatus val);
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
//void HAL_SYSTICK_Callback(void);

/* Private functions ---------------------------------------------------------*/

/** @defgroup SPIRIT1_APPLI_Private_Functions
* @{
*/

/**
* @brief  Initializes RF Transceiver's HAL.
* @param  None
* @retval None.
*/
void HAL_Spirit1_Init(void)
{

  pRadioDriver = &spirit_cb;
  pRadioDriver->Init( ); 
  SpiritSetIntHandler(HAL_GPIO_EXTI_Callback, NULL);
//  
}

void Spirit_ReceiveBuffB(uint8_t *pRxBuff, uint8_t maxBuffLen, uint8_t *cRxlen)
{
  AppliReceiveBuff(pRxBuff, maxBuffLen);
  do
  {
    if((RESET != xRxDoneFlag)||(RESET != rx_timeout)||(SET != exitTime))
    {
      if((rx_timeout==SET)||(exitTime==RESET))
      {
        rx_timeout = RESET;
        AppliReceiveBuff(pRxBuff, maxBuffLen);
      }
      else if(xRxDoneFlag) 
      {
        xRxDoneFlag = RESET;
        pRadioDriver->GetRxPacket(pRxBuff,cRxlen);
        return;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  } while(1);
}

SM_State_t SM_State = SM_STATE_IDLE;/* The actual state running */

/**
* @brief  SPIRIT1 P2P Process State machine
* @param  uint8_t *pTxBuff = Pointer to aTransmitBuffer 
*         uint8_t cTxlen = length of aTransmitBuffer 
*         uint8_t* pRxBuff = Pointer to aReceiveBuffer
*         uint8_t cRxlen= length of aReceiveBuffer
* @retval None.
*/
void P2P_Process(uint8_t *pTxBuff, uint8_t cTxlen, uint8_t* pRxBuff, uint8_t cRxlen)
{
  /*
  uint8_t xIndex = 0;
  uint8_t ledToggleCtr = 0;
  */
  uint8_t dest_addr;
  
  switch(SM_State)
  {
  case SM_STATE_START_RX:
    {   
      AppliReceiveBuff(pRxBuff, cRxlen);
      /* wait for data received or timeout period occured */
      SM_State = SM_STATE_WAIT_FOR_RX_DONE;    
    }
    break;
    
  case SM_STATE_WAIT_FOR_RX_DONE:
    if((RESET != xRxDoneFlag)||(RESET != rx_timeout)||(SET != exitTime))
    {
      if((rx_timeout==SET)||(exitTime==RESET))
      {
        rx_timeout = RESET;
        SM_State = SM_STATE_START_RX;
      }
      else if(xRxDoneFlag) 
      {
        xRxDoneFlag=RESET;
        SM_State = SM_STATE_DATA_RECEIVED;
      }
    }
    break;
    
  case SM_STATE_DATA_RECEIVED:
    {
    //  memset(pRxBuff, 0x00, 96);
      pRadioDriver->GetRxPacket(pRxBuff,&cRxlen);
   
    #if 1
      printf("RCV packet len = %d\n", cRxlen);
      for (int i = 0; i < cRxlen; i++)
      {
        printf("[%d] => %X ", i, pRxBuff[i]); 
      }
      
      printf("\n"); 
    #endif
      /*rRSSIValue = Spirit1GetRssiTH();*/
    #if 0
      xRxFrame.Cmd = pRxBuff[0];
      xRxFrame.CmdLen = pRxBuff[1];
      xRxFrame.Cmdtag = pRxBuff[2];
      xRxFrame.CmdType = pRxBuff[3];
      xRxFrame.DataLen = pRxBuff[4];
      
      for (xIndex = 5; xIndex < cRxlen; xIndex++)
      {
        xRxFrame.DataBuff[xIndex] = pRxBuff[xIndex];
      }
      if(xRxFrame.Cmd == LED_TOGGLE)
      {
        SM_State = SM_STATE_TOGGLE_LED; 
      }
      if(xRxFrame.Cmd == ACK_OK)
      {
        HAL_Delay(DELAY_TX_LED_GLOW); 
        SM_State = SM_STATE_ACK_RECEIVED;
      }
      #endif
      SM_State = SM_STATE_IDLE;
    }
    break;
    
  case SM_STATE_SEND_ACK:
    {
      xTxFrame.Cmd = ACK_OK;
      xTxFrame.CmdLen = 0x01;
      xTxFrame.Cmdtag = xRxFrame.Cmdtag;
      xTxFrame.CmdType = APPLI_CMD;
      xTxFrame.DataBuff = pTxBuff;
      xTxFrame.DataLen = cTxlen;
            
      HAL_Delay(DELAY_TX_LED_GLOW);         
      AppliSendBuff(&xTxFrame);   
      SM_State = SM_STATE_WAIT_FOR_TX_DONE;
    }
    break;   
      
  case SM_STATE_SEND_DATA:
    {
      xTxFrame.Cmd = LED_TOGGLE;
      xTxFrame.CmdLen = 0x01;
      xTxFrame.Cmdtag = txCounter++;
      xTxFrame.CmdType = APPLI_CMD;
      xTxFrame.DataBuff = pTxBuff;
      xTxFrame.DataLen = cTxlen;
      
      AppliSendBuff(&xTxFrame);
 
      SM_State = SM_STATE_WAIT_FOR_TX_DONE;
    }
    break;
    
  case SM_STATE_WAIT_FOR_TX_DONE:
  /* wait for TX done */    
    if(xTxDoneFlag)
    {
      xTxDoneFlag = RESET;
      
      if(xTxFrame.Cmd == LED_TOGGLE)
      {
        SM_State = SM_STATE_START_RX;
      }
      else if(xTxFrame.Cmd == ACK_OK)
      {
        SM_State = SM_STATE_IDLE;  
      }
    }
    break;
    
  case SM_STATE_ACK_RECEIVED:
    SM_State = SM_STATE_IDLE;   
    break;
    
  case SM_STATE_TOGGLE_LED:
      dest_addr = 
      SpiritPktCommonGetReceivedDestAddress();
      
      if ((dest_addr == MULTICAST_ADDRESS) || (dest_addr == BROADCAST_ADDRESS))
      {
        /* in that case do not send ACK to avoid RF collisions between several RF ACK messages */
        HAL_Delay(DELAY_TX_LED_GLOW);
        SM_State = SM_STATE_IDLE;   
      }
      else
      {
        SM_State = SM_STATE_SEND_ACK;  
      }
    break;
    
  case SM_STATE_IDLE:
    SM_State = SM_STATE_START_RX;
    break;
  }
}

void AppliSendBuffAB(AppliFrame_t *xTxFrame, uint8_t dstAdr)
{
  xTxDoneFlag = RESET;
  AppliSendBuffA(xTxFrame, dstAdr);

  while(xTxDoneFlag == RESET)
  {
    HAL_Delay(10);
  }
}

void AppliSendBuffA(AppliFrame_t *xTxFrame, uint8_t dstAdr)
{
  uint8_t xIndex = 0;
  uint8_t trxLength = 0;
  
  SpiritPktBasicAddressesInit(&xAddressInit);
     
  TxFrameBuff[0] = xTxFrame->Cmd;
  TxFrameBuff[1] = xTxFrame->CmdLen;
  TxFrameBuff[2] = xTxFrame->Cmdtag;
  TxFrameBuff[3] = xTxFrame->CmdType;
  TxFrameBuff[4] = xTxFrame->DataLen;
  
  for(; xIndex < xTxFrame->DataLen; xIndex++)
  {
    TxFrameBuff[xIndex+5] =  xTxFrame->DataBuff[xIndex];
  }
  
  
  trxLength = (xIndex+5);
  
  /* Spirit IRQs disable */
  pRadioDriver->DisableIrq();
  /* Spirit IRQs enable */
  pRadioDriver->EnableTxIrq();
  /* payload length config */
  pRadioDriver->SetPayloadLen(trxLength);  
  /* rx timeout config */
  pRadioDriver->SetRxTimeout(RECEIVE_TIMEOUT);
  /* IRQ registers blanking */
  pRadioDriver->ClearIrqStatus();
  /* destination address */
  pRadioDriver->SetDestinationAddress(dstAdr);
  /* send the TX command */
  pRadioDriver->StartTx(TxFrameBuff, trxLength);
}


/**
* @brief  This function handles the point-to-point packet transmission
* @param  AppliFrame_t *xTxFrame = Pointer to AppliFrame_t structure 
*         uint8_t cTxlen = Length of aTransmitBuffer
* @retval None
*/
void AppliSendBuff(AppliFrame_t *xTxFrame)
{
  AppliSendBuffA(xTxFrame, DestinationAddr);
}


/**
* @brief  This function handles the point-to-point packet reception
* @param  uint8_t *RxFrameBuff = Pointer to ReceiveBuffer
*         uint8_t cRxlen = length of ReceiveBuffer
* @retval None
*/
void AppliReceiveBuff(uint8_t *RxFrameBuff, uint8_t cRxlen)
{
  /*float rRSSIValue = 0;*/
  exitTime = SET;
  exitCounter = TIME_TO_EXIT_RX;

  SpiritPktBasicAddressesInit(&xAddressInit);

  SpiritIrqGetStatus(&xIrqStatus);
  /* Spirit IRQs disable */
  pRadioDriver->DisableIrq();
  /* Spirit IRQs enable */
  pRadioDriver->EnableRxIrq();
  /* payload length config */
  pRadioDriver->SetPayloadLen(PAYLOAD_LEN);
  /* rx timeout config */
  pRadioDriver->SetRxTimeout(RECEIVE_TIMEOUT);
  /* destination address */
  pRadioDriver->SetDestinationAddress(DestinationAddr);  
  /* IRQ registers blanking */
  pRadioDriver->ClearIrqStatus(); 
  /* RX command */
  pRadioDriver->StartRx(); 
}

/**
* @brief  This function initializes the protocol for point-to-point 
* communication
* @param  None
* @retval None
*/
void P2P_Init(void)
{
  DestinationAddr=DESTINATION_ADDRESS;
  
  /* Spirit IRQ config */
  pRadioDriver->GpioIrq(&xGpioIRQ);

  /* Spirit Radio config */    
  pRadioDriver->RadioInit(&xRadioInit);

  /* Spirit Radio set power */
  pRadioDriver->SetRadioPower(POWER_INDEX, POWER_DBM);  

  /* Spirit Packet config */  
  pRadioDriver->PacketConfig();
  pRadioDriver->EnableSQI();
  pRadioDriver->SetRssiThreshold(RSSI_THRESHOLD);
}

/**
* @brief  This function initializes the STack Packet handler of spirit1
* @param  None
* @retval None
*/
void STackProtocolInit(void)
{ 
  
   PktStackInit xStackInit=
   {
     .xPreambleLength = PREAMBLE_LENGTH,
     .xSyncLength = SYNC_LENGTH,
     .lSyncWords = SYNC_WORD,
     .xFixVarLength = LENGTH_TYPE,
     .cPktLengthWidth = 8,
     .xCrcMode = CRC_MODE,
     .xControlLength = CONTROL_LENGTH,
     .xFec = EN_FEC,
     .xDataWhitening = EN_WHITENING
   };  
   
   /* Spirit Packet config */
   SpiritPktStackInit(&xStackInit);
}  
                   
/*xDataWhitening;  *
* @brief  This function initializes the BASIC Packet handler of spirit1
* @param  None
* @retval None
*/
void BasicProtocolInit(void)
{
  /* Spirit Packet config */
  SpiritPktBasicInit(&xBasicInit);
  //SpiritPktBasicAddressesInit(&xAddressInit);
}

/**
* @brief  This routine will put the radio and mcu in LPM
* @param  None
* @retval None
*/
void Enter_LP_mode(void)
{  
  pMCU_LPM_Comm = &MCU_LPM_cb;
  pRadio_LPM_Comm = &Radio_LPM_cb;
  
#if defined(MCU_SLEEP_MODE)&&defined(RF_SHUTDOWN) 
  {
    pRadio_LPM_Comm->RadioShutDown(); 
    pMCU_LPM_Comm->McuSleepMode();
  }
#elif defined(MCU_SLEEP_MODE)&&defined(RF_STANDBY) 
  {
    pRadio_LPM_Comm->RadioStandBy(); 
    pMCU_LPM_Comm->McuSleepMode();
  }
#elif defined(MCU_SLEEP_MODE)&&defined(RF_SLEEP) 
  {
    pRadio_LPM_Comm->RadioSleep();
    pMCU_LPM_Comm->McuSleepMode();
  }
#elif defined(MCU_STOP_MODE)&&defined(RF_SHUTDOWN) 
  {
    pRadio_LPM_Comm->RadioShutDown(); 
    pMCU_LPM_Comm->McuStopMode();
  }
#elif defined(MCU_STOP_MODE)&&defined(RF_STANDBY) 
  {
    pRadio_LPM_Comm->RadioStandBy(); 
    pMCU_LPM_Comm->McuStopMode();
  }
#elif defined(MCU_STOP_MODE)&&defined(RF_SLEEP) 
  {
    pRadio_LPM_Comm->RadioSleep();
    pMCU_LPM_Comm->McuStopMode();
  }
#else
  pMCU_LPM_Comm->McuSleepMode();
#endif
}

/**
* @brief  This routine wake-up the mcu and radio from LPM
* @param  None
* @retval None
*/
void Exit_LP_mode(void)
{
  pRadio_LPM_Comm = &Radio_LPM_cb;      
  pRadio_LPM_Comm->RadioPowerON();  
}

/**
* @brief  This routine puts the MCU in stop mode
* @param  None
* @retval None
*/
void MCU_Enter_StopMode(void)
{
 // APPLI_EnterSTOPMode();
}

/**
* @brief  This routine puts the MCU in standby mode
* @param  None
* @retval None
*/
void MCU_Enter_StandbyMode(void)
{
  //APPLI_EnterSTANDBYMode();
}

/**
* @brief  This routine puts the MCU in sleep mode
* @param  None
* @retval None
*/
void MCU_Enter_SleepMode(void)
{
  /*Suspend Tick increment to prevent wakeup by Systick interrupt. 
  Otherwise the Systick interrupt will wake up the device within 1ms (HAL time base)*/

  //APPLI_EnterSLEEPMode();
}

/**
* @brief  This function will turn on the radio and waits till it enters the Ready state.
* @param  Param:None. 
* @retval None
*                       
*/
void RadioPowerON(void)
{
  SpiritCmdStrobeReady();   
  do{
    /* Delay for state transition */
    for(volatile uint8_t i=0; i!=0xFF; i++);
    
    /* Reads the MC_STATUS register */
    SpiritRefreshStatus();
  }
  while(g_xStatus.MC_STATE!=MC_STATE_READY);
}


/**
* @brief  This function will Shut Down the radio.
* @param  Param:None. 
* @retval None
*                       
*/
void RadioPowerOFF(void)
{
  SpiritEnterShutdown();
}


/**
* @brief  This function will put the radio in standby state.
* @param  None. 
* @retval None
*                       
*/
void RadioStandBy(void)
{
  SpiritCmdStrobeStandby();  
}

/**
* @brief  This function will put the radio in sleep state.
* @param  None. 
* @retval None
*                       
*/
void RadioSleep(void)
{
  SpiritCmdStrobeSleep(); 
}

/**
* @brief  This routine updates the respective status for key press.
* @param  None
* @retval None
*/
void Set_KeyStatus(FlagStatus val)
{
  if(val==SET)
  {
    SM_State = SM_STATE_SEND_DATA;
  }
}

/**
* @brief  This function handles External interrupt request. In this application it is used
*         to manage the Spirit IRQ configured to be notified on the Spirit GPIO_3.
* @param  None
* @retval None
*/
void P2PInterruptHandler(void)
{
  
  SpiritIrqGetStatus(&xIrqStatus);
 
  /* Check the SPIRIT TX_DATA_SENT IRQ flag */
  if(
     (xIrqStatus.IRQ_TX_DATA_SENT)        
       ||(xIrqStatus.IRQ_MAX_BO_CCA_REACH)
    )
  {
      SpiritCsma(S_DISABLE);  
      SpiritRadioPersistenRx(S_ENABLE);
      SpiritRadioCsBlanking(S_ENABLE);
      
      if(xIrqStatus.IRQ_MAX_BO_CCA_REACH)
      {
        SpiritCmdStrobeSabort();
      }

      SpiritQiSetRssiThresholddBm(RSSI_THRESHOLD);

      xTxDoneFlag = SET;
  }
  
  /* Check the SPIRIT RX_DATA_READY IRQ flag */
  if((xIrqStatus.IRQ_RX_DATA_READY))
  {
    xRxDoneFlag = SET;   
  }
  
  /* Restart receive after receive timeout*/
  if (xIrqStatus.IRQ_RX_TIMEOUT)
  {
    rx_timeout = SET; 
    SpiritCmdStrobeRx();
  }
  
  /* Check the SPIRIT RX_DATA_DISC IRQ flag */
  if(xIrqStatus.IRQ_RX_DATA_DISC)
  {      
    /* RX command - to ensure the device will be ready for the next reception */
    SpiritCmdStrobeRx();
  }  
}


/**
* @}
*/

/**
  * @brief GPIO EXTI callback
  * @param uint16_t GPIO_Pin
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(void *arg)
{
  P2PInterruptHandler();
}
/**
* @}
*/

/**
* @}
*/

/**
* @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
