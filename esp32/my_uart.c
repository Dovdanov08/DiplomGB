/*
 * uart.c
 *
 *  Created on: 6 янв. 2024 г.
 *      Author: Intel
 */
#include "my_uart.h"

extern char *TAG;

void uart_init(void)
{
	const uart_config_t uart_config = {
		.baud_rate = 115200,
		.data_bits = UART_DATA_8_BITS,
		.parity = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
		.source_clk = UART_SCLK_DEFAULT,
	};
	// We won't use a buffer for sending data.
	uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
	uart_param_config(UART_NUM_1, &uart_config);
	uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

	ESP_LOGI(TAG, "Example configured to blink GPIO LED!");
	gpio_reset_pin(LED_GPIO);
	/* Set the GPIO as a push/pull output */
	gpio_set_direction(LED_GPIO , GPIO_MODE_OUTPUT);

}
int uart_sendData(const char* logName, uint8_t* data, uint8_t lendata)
{
    const int txBytes = uart_write_bytes(UART_NUM_1, data, lendata);
    //ESP_LOGI(logName, "Wrote %d bytes %d", txBytes, *data);
    return txBytes;
}
/*
static void tx_task(void *arg)
{
	uint8_t data;
	BaseType_t Status=pdPASS;
	static const char *TX_TASK_TAG = "TX_TASK";
	esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);
	gpio_set_level(LED_GPIO, 0);
	while (1) {
		if(Status==xQueueReceive(Data_queue, &data, 100))
		{
			sendData(TX_TASK_TAG, &data,sizeof(data));
			gpio_set_level(LED_GPIO, 1);
		}
		vTaskDelay(250 / portTICK_PERIOD_MS);
		gpio_set_level(LED_GPIO, 0);
	}
}

static void rx_task(void *arg)
{
	static const char *RX_TASK_TAG = "RX_TASK";
	esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
	uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE + 1);
	float data_f = 0.0;
	float data_f1 = 0.0;

	while (1) {
		const int rxBytes = uart_read_bytes(UART_NUM_1, data, RX_BUF_SIZE, 1000 / portTICK_PERIOD_MS);
		if ((rxBytes > 0) && (rxBytes < 4))
		{
				data[rxBytes] = 0;
				//ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, data);
				ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);
		}
		else
		{
			for(int i=0; i<rxBytes;i++)
			{
				*((uint8_t*)&data_f+i)=*(data+i);
			}
			for(int i=0; i<rxBytes;i++)
			{
				*((uint8_t*)&data_f1+i)=*(data+sizeof(data_f1)+i);
			}
			ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%.2f' '%.2f'", rxBytes, data_f, data_f1);
			//ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);
			//ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, ptrfloat, rxBytes, ESP_LOG_INFO);
			//ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, &data_f, rxBytes, ESP_LOG_INFO);
		}
	}
	free(data);
}*/

