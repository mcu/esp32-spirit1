/* Includes ------------------------------------------------------------------*/
#include "radio_gpio.h"
#include "esp_log.h"
#include "soc/gpio_struct.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"

#define RADIO_GPIO_0_NUM    12
#define RADIO_GPIO_1_NUM    21
#define RADIO_GPIO_2_NUM    15
#define RADIO_GPIO_3_NUM    34
#define RADIO_GPIO_SDN_NUM  33

typedef struct 
{
  uint32_t pin;
  gpio_isr_t isr_handler;
  void *arg;
  bool isIsr;
} tRadioGpio;

static tRadioGpio radioGpio_num[RADIO_GPIO_NUMBER] =
{
  { .pin = RADIO_GPIO_0_NUM, .isr_handler = NULL,},
  { .pin = RADIO_GPIO_1_NUM, .isr_handler = NULL,},
  { .pin = RADIO_GPIO_2_NUM, .isr_handler = NULL,},
  { .pin = RADIO_GPIO_3_NUM, .isr_handler = NULL,},
  { .pin = RADIO_GPIO_SDN_NUM,.isr_handler = NULL,},
};

static bool isInit;
static xQueueHandle queue;

/**
 * @defgroup Radio_Gpio_Private_Macros                 Radio_Gpio Private Macros
 * @{
 */
#define POR_TIME ((uint16_t)0x1E00)
void RadioExitShutdown(void);
void RadioEnterShutdown(void);

FlagStatus RadioGpioGetLevel(RadioGpioPin xGpio);
void RadioGpioInterruptCmd(RadioGpioPin xGpio, uint8_t nPreemption, uint8_t nSubpriority, FunctionalState xNewState);
void RadioGpioInit(RadioGpioPin xGpio, RadioGpioMode xGpioMode);
FlagStatus RadioCheckShutdown(void);


/**
 * @}
 */


/**
 * @defgroup Radio_Gpio_Private_Functions              Radio_Gpio Private Functions
 * @{
 */
 

/**
* @brief  Configures MCU GPIO and EXTI Line for GPIOs.
* @param  xGpio Specifies the GPIO to be configured.
*         This parameter can be one of following parameters:
*         @arg GPIO_0
*         @arg GPIO_1
*         @arg GPIO_2
*         @arg GPIO_3
* @param  xGpioMode Specifies GPIO mode.
*         This parameter can be one of following parameters:
*         @arg RADIO_MODE_GPIO_IN: MCU GPIO will be used as simple input.
*         @argRADIO_MODE_GPIO_OUT: MCU GPIO will be used as simple output.
*         @arg RADIO_MODE_EXTI_IN: MCU GPIO will be connected to EXTI line with interrupt
*         generation capability.
* @retval None.
*/
void RadioGpioInit(RadioGpioPin xGpio, RadioGpioMode xGpioMode)
{
    gpio_config_t gpcfg = {0};

  //  
    switch (xGpioMode)
    {
      case RADIO_MODE_GPIO_IN:
        gpcfg.mode = GPIO_MODE_INPUT;
        break;

      case RADIO_MODE_GPIO_OUT:
        gpcfg.mode = GPIO_MODE_OUTPUT;
        break;

      case RADIO_MODE_EXTI_IN:
        gpcfg.mode = GPIO_MODE_INPUT;
        gpcfg.intr_type = GPIO_INTR_NEGEDGE;
        break;
      
      default:
        break;
    }

    gpcfg.pull_up_en = GPIO_PULLUP_ENABLE;
   
    gpcfg.pin_bit_mask = 1ULL<<radioGpio_num[xGpio].pin;
    gpio_config(&gpcfg);
}

static void radio_isr_task(void *arg)
{
  tRadioGpio *xGpio = 0;
  while(1)
  {
    while (xQueueReceive(queue, &xGpio, portMAX_DELAY))
    {
      if (xGpio->isr_handler)
      {
        xGpio->isr_handler(xGpio->arg); 
      }
    }

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

static void IRAM_ATTR gpio_isr_handler(void *arg)
{
  tRadioGpio *gpio = (tRadioGpio *)arg;
  if (!arg)
  {
    return;
  }
  xQueueSendFromISR(queue, &gpio, NULL);  
}

static void  RadioGpioInterruptInit(void)
{
  queue = xQueueCreate(10, sizeof(tRadioGpio *));
  #if 1
    gpio_install_isr_service(ESP_INTR_FLAG_EDGE);
  #endif
  xTaskCreate(radio_isr_task, "RadioIsrGpioTask", 2 * 1024, NULL, 15, NULL);
}

void RadioGpioInterruptSetHandler(RadioGpioPin xGpio, void (*hnd)(void *), void *arg)
{

  if (!isInit)
  {
    RadioGpioInterruptInit();
    isInit = true;
  }
  
  radioGpio_num[xGpio].isr_handler = hnd;
  radioGpio_num[xGpio].arg = arg;
  gpio_isr_handler_remove(radioGpio_num[xGpio].pin);

  if (NULL != hnd)
  {
    gpio_isr_handler_add(radioGpio_num[xGpio].pin, gpio_isr_handler, &radioGpio_num[xGpio]);
  }
}


/**
* @brief  Enables or disables the interrupt on GPIO .
* @param  xGpio Specifies the GPIO whose priority shall be changed.
*         This parameter can be one of following parameters:
*         @arg GPIO_0
*         @arg GPIO_1
*         @arg GPIO_2
*         @arg GPIO_3
* @param  nPreemption Specifies Preemption Priority.
* @param  nSubpriority Specifies Subgroup Priority.
* @param  xNewState Specifies the State.
*         This parameter can be one of following parameters:
*         @arg ENABLE: Interrupt is enabled
*         @arg DISABLE: Interrupt is disabled
* @retval None.
*/
void RadioGpioInterruptCmd(RadioGpioPin xGpio, uint8_t nPreemption, uint8_t nSubpriority, FunctionalState xNewState)
{

}


/**
* @brief  Returns the level of a specified GPIO.
* @param  xGpio Specifies the GPIO to be read.
*         This parameter can be one of following parameters:
*         @arg GPIO_0
*         @arg GPIO_1
*         @arg GPIO_2
*         @arg GPIO_3
* @retval FlagStatus Level of the GPIO. This parameter can be:
*         SET or RESET.
*/
FlagStatus RadioGpioGetLevel(RadioGpioPin xGpio)
{
  /* Gets the GPIO level */
  return gpio_get_level(radioGpio_num[xGpio].pin)?SET:RESET;
}


/**
* @brief  Sets the level of a specified GPIO.
* @param  xGpio Specifies the GPIO to be set.
*         This parameter can be one of following parameters:
*         @arg GPIO_0
*         @arg GPIO_1
*         @arg GPIO_2
*         @arg GPIO_3
* @param  GPIO_PinState Level of the GPIO. This parameter can be:
*         GPIO_PIN_SET or GPIO_PIN_RESET.
* @retval None.
*/
void RadioGpioSetLevel(RadioGpioPin xGpio, GPIO_PinState xState)
{
  /* Sets the GPIO level */
  gpio_set_level(radioGpio_num[xGpio].pin, xState);
}


/**
* @brief  Puts at logic 1 the SDN pin.
* @param  None.
* @retval None.
*/
void RadioEnterShutdown(void)
{
  /* Puts high the GPIO connected to shutdown pin */
  /* Check the parameters */ 
  RadioGpioSetLevel(RADIO_GPIO_SDN, SET);
  vTaskDelay(pdMS_TO_TICKS(2));
}



/**
* @brief  Put at logic 0 the SDN pin.
* @param  None.
* @retval None.
*/
void RadioExitShutdown(void)
{
  /* Puts low the GPIO connected to shutdown pin */
  RadioGpioSetLevel(RADIO_GPIO_SDN, RESET);

  /* Delay to allow the circuit POR, about 700 us */
  vTaskDelay(pdMS_TO_TICKS(2));
}


/**
* @brief  check the logic(0 or 1) at the SDN pin.
* @param  None.
* @retval FlagStatus.
*/
FlagStatus RadioCheckShutdown(void)
{
  return RadioGpioGetLevel(RADIO_GPIO_SDN);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/