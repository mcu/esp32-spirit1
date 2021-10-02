#include <string.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "soc/gpio_struct.h"
#include "driver/gpio.h"
#include "MCU_Interface.h"
#include "radio_gpio.h"

#define WRITE_MASK   0x00
#define READ_MASK    0x01
#define COMMAND_MASK 0x80

#define PIN_NUM_MOSI 19
#define PIN_NUM_MISO 18
#define PIN_NUM_CLK  14
#define PIN_NUM_CS   13

#define LINEAR_FIFO_ADDRESS 0xFF

static spi_device_handle_t spi; 
static SemaphoreHandle_t spi_sem;

static void RadioCsSetLevel(uint32_t level);
static StatusBytes RadioSpiRWRegisters(uint8_t mask, uint8_t cRegAddress,
		uint8_t cNbBytes, uint8_t* pcBuffer);

void RadioSpiInit(void)
{
    gpio_config_t gpcfg = {0};
	esp_err_t ret;

    spi_bus_config_t buscfg={
        .miso_io_num=PIN_NUM_MISO,
        .mosi_io_num=PIN_NUM_MOSI,
        .sclk_io_num=PIN_NUM_CLK,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1
    };
    spi_device_interface_config_t devcfg={
        .clock_speed_hz=1000*1000,             //Clock out at 1 MHz
        .mode=0,                                //SPI mode 0
        .spics_io_num=-1,                       //CS pin - software
        .queue_size=1,                          //We want to be able to queue 1 transactions at a tie
    };

    spi_sem = xSemaphoreCreateMutex();
    gpio_set_pull_mode(PIN_NUM_MOSI, GPIO_PULLDOWN_ONLY); 
    gpio_set_pull_mode(PIN_NUM_MISO, GPIO_PULLDOWN_ONLY); 
    gpio_set_pull_mode(PIN_NUM_CLK,  GPIO_PULLUP_ONLY); 

    gpcfg.mode = GPIO_MODE_OUTPUT;
    gpcfg.pin_bit_mask = 1ULL<<PIN_NUM_CS;
    gpcfg.pull_up_en   = GPIO_PULLUP_ENABLE;
    gpcfg.pull_down_en = GPIO_PULLDOWN_ENABLE;
    gpio_config(&gpcfg);

 	ret=spi_bus_initialize(HSPI_HOST, &buscfg, 1);
    assert(ret==ESP_OK);
    ret=spi_bus_add_device(HSPI_HOST, &devcfg, &spi);
    assert(ret==ESP_OK);

    RadioCsSetLevel(1);
}

StatusBytes RadioSpiWriteRegisters(uint8_t cRegAddress, uint8_t cNbBytes, uint8_t* pcBuffer)
{
	return RadioSpiRWRegisters(WRITE_MASK, cRegAddress, cNbBytes, pcBuffer);
}

StatusBytes RadioSpiReadRegisters(uint8_t cRegAddress, uint8_t cNbBytes, uint8_t* pcBuffer)
{
	return RadioSpiRWRegisters(READ_MASK, cRegAddress, cNbBytes, pcBuffer);
}

StatusBytes RadioSpiCommandStrobes(uint8_t cCommandCode)
{
	return RadioSpiRWRegisters(COMMAND_MASK, cCommandCode, 0, NULL);
}

StatusBytes RadioSpiWriteFifo(uint8_t cNbBytes, uint8_t* pcBuffer)
{
	return RadioSpiRWRegisters(WRITE_MASK, LINEAR_FIFO_ADDRESS, cNbBytes, pcBuffer);
}

StatusBytes RadioSpiReadFifo(uint8_t cNbBytes, uint8_t* pcBuffer)
{
	return RadioSpiRWRegisters(READ_MASK, LINEAR_FIFO_ADDRESS, cNbBytes, pcBuffer);
}

void EnterShutdown(void)
{
	RadioEnterShutdown();
}

void ExitShutdown(void)
{
	RadioExitShutdown();
}

SpiritFlagStatus CheckShutdown(void)
{
	return (RadioCheckShutdown() == SET)?S_SET:S_RESET;
}

static void RadioCsSetLevel(uint32_t level)
{
	gpio_set_level(PIN_NUM_CS, level);
			vTaskDelay(pdMS_TO_TICKS(1));

}

static StatusBytes RadioSpiRWRegisters(uint8_t mask, uint8_t cRegAddress, uint8_t cNbBytes, uint8_t* pcBuffer)
{
	esp_err_t ret;
	spi_transaction_t t;
	uint8_t command_hdr[2];
	uint16_t tmp_st = 0;
	StatusBytes St;

	while(!xSemaphoreTake(spi_sem, portMAX_DELAY))
	{
		vTaskDelay(pdMS_TO_TICKS(1));
	}

	RadioCsSetLevel(0);
	memset(&t, 0, sizeof(t));

	command_hdr[0] = mask;
	command_hdr[1] = cRegAddress;

	t.length    = 8 * 2;                   
    t.tx_buffer = command_hdr;              
    t.rx_buffer = &tmp_st;

    ret=spi_device_transmit(spi, &t);
    assert(ret==ESP_OK);

    tmp_st = (tmp_st<<8) | (tmp_st>>8);

	memcpy(&St, &tmp_st, sizeof(StatusBytes));

	if ((COMMAND_MASK != mask) && (cNbBytes))
	{	
		memset(&t, 0, sizeof(t));
		t.length    = 8 * cNbBytes; 
		if (READ_MASK == mask)
		{
			t.rx_buffer = pcBuffer;           
		} else
		{          
			t.tx_buffer = pcBuffer;
		}
	
	    ret=spi_device_transmit(spi, &t);	
	    assert(ret==ESP_OK);
	    
	}
    
	RadioCsSetLevel(1);	
	xSemaphoreGive(spi_sem);
	return St;
}
