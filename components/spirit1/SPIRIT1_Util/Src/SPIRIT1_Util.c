/* Includes ------------------------------------------------------------------*/
#include "esp_log.h"
#include "SPIRIT1_Util.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "spirit1_appli.h"
//#endif
/**
* @addtogroup ST_SPIRIT1
* @{
*/


/**
* @addtogroup SPIRIT1_Util
* @{
*/


/**
* @defgroup SPIRIT1_Util_Private_TypesDefinitions       SPIRIT1_Util Private Types Definitions
* @{
*/

/**
* @}
*/


/**
* @defgroup SPIRIT1_Util_Private_Defines                SPIRIT1_Util Private Defines
* @{
*/

/**
* @}
*/


/**
* @defgroup SPIRIT1_Util_Private_Macros                 SPIRIT1_Util Private Macros
* @{
*/
#define XTAL_FREQUENCY          50000000L
#define SPIRIT_VERSION          SPIRIT_VERSION_3_0
#define RANGE_TYPE              RANGE_EXT_NONE       /*RANGE_EXT_SKYWORKS*/
/**
* @}
*/


/**
* @defgroup SPIRIT1_Util_Private_Variables              SPIRIT1_Util Private Variables
* @{
*/

/**
* @brief A map that contains the SPIRIT version
*/
const SpiritVersionMap xSpiritVersionMap[] =
{
  /* The Control Board frame handler functions */
  {CUT_2_1v4, SPIRIT_VERSION_2_1},
  {CUT_2_1v3, SPIRIT_VERSION_2_1},
  {CUT_3_0, SPIRIT_VERSION_3_0},
};
static RangeExtType xRangeExtType = RANGE_EXT_NONE;
static uint8_t s_RfModuleBand = 0;
static uint8_t s_eeprom = 0;
static int32_t s_RfModuleOffset=0;


/**
* @}
*/


/**
* @defgroup SPIRIT1_Util_Private_FunctionPrototypes     SPIRIT1_Util Private Function Prototypes
* @{
*/

/**
* @}
*/


/**
* @defgroup SPIRIT1_Util_Private_Functions              SPIRIT1_Util Private Functions
* @{
*/

/**
* @brief  Read the status register.
* @param  None
* @retval Status
*/
void Spirit1InterfaceInit(void)
{ 

  RadioGpioInit(RADIO_GPIO_SDN,RADIO_MODE_GPIO_OUT);

  SpiritSpiInit();
  
     
  /* Board management */   
  SpiritEnterShutdown(); 
  SpiritExitShutdown();   
  SpiritManagementIdentificationRFBoard();

  SpiritManagementRangeExtInit(); 
  
      
  RadioGpioInit(RADIO_GPIO_3,RADIO_MODE_EXTI_IN);
}

void SpiritSetIntHandler(void (*handler)(void *), void *arg)
{
  RadioGpioInterruptSetHandler(RADIO_GPIO_3, handler, arg);
}



#if defined(SPIRIT1_HAS_EEPROM)
/**
* @brief  Read the status register.
* @param  None
* @retval Status
*/
uint8_t EepromIdentification(void)
{
  uint8_t status;
  status = EepromSetSrwd();
  status = EepromStatus();
  if((status&0xF0) == EEPROM_STATUS_SRWD) 
  { 
    /*0xF0 mask [SRWD 0 0 0]*/
    status = 1;
    EepromResetSrwd();
  }
  else
    status = 0;
  
  return status;
}
#endif

/**
* @brief  Identifies the SPIRIT1 Xtal frequency and version.
* @param  None
* @retval Status
*/
void SpiritManagementIdentificationRFBoard(void)
{
  do{
    /* Delay for state transition */
    vTaskDelay(pdMS_TO_TICKS(1));

    /* Reads the MC_STATUS register */
    SpiritRefreshStatus();

  }while(g_xStatus.MC_STATE!=MC_STATE_READY);

  SpiritRadioSetXtalFrequency(XTAL_FREQUENCY);        
  //SpiritGeneralSetSpiritVersion(SPIRIT_VERSION); 
}



/**
* @brief  Sets the SPIRIT frequency band
* @param  uint8_t value: RF FREQUENCY
* @retval None
*/
void SpiritManagementSetBand(uint8_t value)
{
  s_RfModuleBand = value;
}


/**
* @brief  returns the SPIRIT frequency band
* @param  None
* @retval uint8_t value: RF FREQUENCY
*/
uint8_t SpiritManagementGetBand(void)
{
  return s_RfModuleBand;
}

/**
* @defgroup RANGE_EXT_MANAGEMENT_FUNCTIONS              SDK SPIRIT Management Range Extender Functions
* @{
*/
void SpiritManagementRangeExtInit(void)
{
  RangeExtType range_type = SpiritManagementGetRangeExtender();
  
  if(range_type==RANGE_EXT_SKYWORKS_169) {
    /* TCXO optimization power consumption */
    SpiritGeneralSetExtRef(MODE_EXT_XIN);
    uint8_t tmp = 0x01; SpiritSpiWriteRegisters(0xB6,1,&tmp);
    
    /* CSD control */
    SpiritGpioInit(&(SGpioInit){SPIRIT_GPIO_0, SPIRIT_GPIO_MODE_DIGITAL_OUTPUT_HP, SPIRIT_GPIO_DIG_OUT_TX_RX_MODE});
    
    /* CTX/BYP control */
    SpiritGpioInit(&(SGpioInit){SPIRIT_GPIO_1, SPIRIT_GPIO_MODE_DIGITAL_OUTPUT_HP, SPIRIT_GPIO_DIG_OUT_TX_STATE});
    
    /* Vcont control */
    SpiritGpioInit(&(SGpioInit){SPIRIT_GPIO_2, SPIRIT_GPIO_MODE_DIGITAL_OUTPUT_HP, SPIRIT_GPIO_DIG_OUT_RX_STATE});
  }
  else if(range_type==RANGE_EXT_SKYWORKS_868) {   
    /* CSD control */
    SpiritGpioInit(&(SGpioInit){SPIRIT_GPIO_0, SPIRIT_GPIO_MODE_DIGITAL_OUTPUT_HP, SPIRIT_GPIO_DIG_OUT_TX_RX_MODE});
    
    /* CTX/BYP control */
    SpiritGpioInit(&(SGpioInit){SPIRIT_GPIO_1, SPIRIT_GPIO_MODE_DIGITAL_OUTPUT_HP, SPIRIT_GPIO_DIG_OUT_RX_STATE});
    
    /* Vcont control */
    SpiritGpioInit(&(SGpioInit){SPIRIT_GPIO_2, SPIRIT_GPIO_MODE_DIGITAL_OUTPUT_HP, SPIRIT_GPIO_DIG_OUT_TX_STATE});
  }
}

/**
* @brief  returns the spirit1 range extender type
* @param  None
* @retval RangeExtType
*/
RangeExtType SpiritManagementGetRangeExtender(void)
{
  return xRangeExtType;
}

/**
* @brief  Sets the spirit1 range extender type
* @param  RangeExtType
* @retval None
*/
void SpiritManagementSetRangeExtender(RangeExtType xRangeType)
{
  xRangeExtType = xRangeType;
}

/**
* @brief  this function returns the value to indicate that EEPROM is present or not
* @param  None
* @retval uint8_t: 0 or 1
*/
uint8_t SdkEvalGetHasEeprom(void)
{
  return s_eeprom;
}

/**
* @brief  this function setc the value to indicate that EEPROM is present or not
* @param  None
* @retval uint8_t: 0 or 1
*/
void SdkEvalSetHasEeprom(uint8_t eeprom)
{
  s_eeprom = eeprom;
}

/**
* @brief  this function intializes the spirit1 gpio irq for TX and Rx
* @param  None
* @retval None
*/
void Spirit1GpioIrqInit(SGpioInit *pGpioIRQ)
{
  /* Spirit IRQ config */
  SpiritGpioInit(pGpioIRQ);
}

/**
* @brief  this function used to receive RX packet
* @param  None
* @retval None
*/
void Spirit1RadioInit(SRadioInit *pRadioInit)
{    
  /* Spirit Radio config */
  SpiritRadioInit(pRadioInit);

}

/**
* @brief  this function sets the radio power
* @param  uint8_t cIndex, float fPowerdBm
* @retval None
*/
void Spirit1SetPower(uint8_t cIndex, float fPowerdBm)
{
  /* Spirit Radio set power */
  SpiritRadioSetPALeveldBm(cIndex,fPowerdBm);
  SpiritRadioSetPALevelMaxIndex(cIndex);
}

/**
* @brief  this function sets the packet configuration according to the protocol used
* @param  None
* @retval None
*/
void Spirit1PacketConfig(void)
{ 
  BasicProtocolInit();
}

/**
* @brief  this function sets the payload length
* @param  uint8_t length
* @retval None
*/
void Spirit1SetPayloadlength(uint8_t length)
{
  /* payload length config */
  SpiritPktBasicSetPayloadLength(length);
}

/**
* @brief  this function sets the destination address
* @param  uint8_t adress
* @retval None
*/
void Spirit1SetDestinationAddress(uint8_t address)
{
  /* destination address */
  SpiritPktBasicSetDestinationAddress(address);
}

/**
* @brief  this function enables the Tx IRQ
* @param  None
* @retval None
*/
void Spirit1EnableTxIrq(void)
{
  /* Spirit IRQs enable */
  SpiritIrq(TX_DATA_SENT, S_ENABLE); 
}

/**
* @brief  this function enables the Rx IRQ
* @param  None
* @retval None
*/
void Spirit1EnableRxIrq(void)
{
    /* Spirit IRQs enable */
  SpiritIrq(RX_DATA_READY, S_ENABLE);
  SpiritIrq(RX_DATA_DISC, S_ENABLE); 
  SpiritIrq(RX_TIMEOUT, S_ENABLE);
}

/**
* @brief  this function disable IRQs
* @param  None
* @retval None
*/
void Spirit1DisableIrq(void)
{
  /* Spirit IRQs enable */
  SpiritIrqDeInit(NULL);
}
/**
* @brief  this function set the receive timeout period
* @param  None
* @retval None
*/
void Spirit1SetRxTimeout(float cRxTimeOut)
{
  if(cRxTimeOut == 0)
  {
    /* rx timeout config */
    SET_INFINITE_RX_TIMEOUT();
    SpiritTimerSetRxTimeoutStopCondition(ANY_ABOVE_THRESHOLD);
  }
  else
  {
    /* RX timeout config */
    SpiritTimerSetRxTimeoutMs(cRxTimeOut);
    Spirit1EnableSQI();
    SpiritTimerSetRxTimeoutStopCondition(RSSI_AND_SQI_ABOVE_THRESHOLD);  
  }
}

/**
* @brief  this function sets the RSSI threshold
* @param  int dbmValue
* @retval None
*/
void Spirit1SetRssiTH(int dbmValue)
{
  SpiritQiSetRssiThresholddBm(dbmValue);
}

/**
* @brief  this function sets the RSSI threshold
* @param  int dbmValue
* @retval None
*/
float Spirit1GetRssiTH(void)
{
  float dbmValue=0;
  dbmValue = SpiritQiGetRssidBm();
  return dbmValue;
}

/**
* @brief  this function enables SQI check
* @param  None
* @retval None
*/
void Spirit1EnableSQI(void)
{
  /* enable SQI check */
  SpiritQiSetSqiThreshold(SQI_TH_0);
  SpiritQiSqiCheck(S_ENABLE);
}

/**
* @brief  this function starts the RX process
* @param  None
* @retval None
*/
void Spirit1StartRx(void)
{
#if 1
  if(g_xStatus.MC_STATE==MC_STATE_RX)
  {
    SpiritCmdStrobeSabort();
  }
  #endif
 /* RX command */
  SpiritCmdStrobeRx();
}

/**
* @brief  this function receives the data
* @param  None
* @retval None
*/
void Spirit1GetRxPacket(uint8_t *buffer, uint8_t *cRxData )
{
  uint8_t noofbytes = 0;
  /* when rx data ready read the number of received bytes */
  *cRxData=SpiritLinearFifoReadNumElementsRxFifo();
  noofbytes = *cRxData;

  /* read the RX FIFO */
  SpiritSpiReadLinearFifo(noofbytes, buffer);
  
  SpiritCmdStrobeFlushRxFifo();
}

/**
* @brief  this function starts the TX process
* @param  None
* @retval None
*/
void Spirit1StartTx(uint8_t *buffer, uint8_t size )
{
  if(g_xStatus.MC_STATE==MC_STATE_RX)
  {
    SpiritCmdStrobeSabort();
  }
  
#ifdef CSMA_ENABLE
  
    /* Enable CSMA */
    SpiritRadioPersistenRx(S_DISABLE);
    SpiritRadioCsBlanking(S_DISABLE);

    SpiritCsmaInit(&xCsmaInit);
    SpiritCsma(S_ENABLE);
    SpiritQiSetRssiThresholddBm(CSMA_RSSI_THRESHOLD);
    
#endif 
  
  /* fit the TX FIFO */
  SpiritCmdStrobeFlushTxFifo();
  
  SpiritSpiWriteLinearFifo(size, buffer);
  
  /* send the TX command */
  SpiritCmdStrobeTx();
}

/**
* @brief  this function clear the IRQ status
* @param  None
* @retval None
*/
void Spirit1ClearIRQ(void)
{
  SpiritIrqClearStatus();
}


void SpiritManagementSetOffset(int32_t value)
{
  s_RfModuleOffset=value;
}

int32_t SpiritManagementGetOffset(void)
{
  return s_RfModuleOffset;
}




/**
* @}
*/

/**
* @}
*/


/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
