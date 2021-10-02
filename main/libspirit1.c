#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "spirit1_appli.h"

#define TAG "SPIRIT"

#define TX_BUFFER_SIZE   96
#define RX_BUFFER_SIZE   96
#define SPIRIT_OVERHEAD_SIZE 5

static uint8_t RxLength = RX_BUFFER_SIZE;
static uint8_t aTransmitBuffer[TX_BUFFER_SIZE] = {0x00};
static uint8_t aReceiveBuffer[RX_BUFFER_SIZE] = {0x00};
static AppliFrame_t xTxFrame;

void spirit_send_buff(uint8_t *buff, uint8_t buffLen, uint8_t dstAdr)
{
	memcpy(aTransmitBuffer, buff, buffLen);
	xTxFrame.Cmd = LED_TOGGLE;
    xTxFrame.CmdLen = 0x01;
    xTxFrame.Cmdtag = 8;
    xTxFrame.CmdType = APPLI_CMD;
    xTxFrame.DataBuff = aTransmitBuffer;
    xTxFrame.DataLen = buffLen;
      
    AppliSendBuffAB(&xTxFrame, dstAdr);
}

uint8_t spirit_get_buff(uint8_t *buff, uint8_t buffLen)
{
	Spirit_ReceiveBuffB(aReceiveBuffer, RX_BUFFER_SIZE, &RxLength);
	if (RxLength <= (buffLen + SPIRIT_OVERHEAD_SIZE))
	{
		RxLength -= SPIRIT_OVERHEAD_SIZE;
		memcpy(buff, &aReceiveBuffer[SPIRIT_OVERHEAD_SIZE], RxLength);
	}

	return RxLength;
}

void spirit_init(void)
{
    HAL_Spirit1_Init();
    P2P_Init();
}
