#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "spirit1_appli.h"

#define TAG "P2P"

/* Private typedef -----------------------------------------------------------*/ 
/* Private define ------------------------------------------------------------*/
#define TX_BUFFER_SIZE   96
#define RX_BUFFER_SIZE   96

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t TxLength = TX_BUFFER_SIZE;
uint8_t RxLength = RX_BUFFER_SIZE;
uint8_t aTransmitBuffer[TX_BUFFER_SIZE] = {0x00};
uint8_t aReceiveBuffer[RX_BUFFER_SIZE] = {0x00};

void app_main()
{
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_LOGI(TAG, "HAL...");
    HAL_Spirit1_Init();

    ESP_LOGI(TAG, "P2P...");
    P2P_Init();

    ESP_LOGI(TAG, "Running...");
    for(;;)
    {
        P2P_Process(aTransmitBuffer, TxLength, aReceiveBuffer, RxLength);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
