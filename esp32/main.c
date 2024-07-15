#include "main.h"

const char *TAG= "mqtt_start";

QueueHandle_t Command_queue 	= 	NULL;
QueueHandle_t Float_queue 	=	NULL;
QueueHandle_t Data_queue 	=	NULL;
//QueueHandle_t mqtt_queue 	=	NULL;
typedef struct
{
	float		temperature;
	float 		humidity;
	uint8_t		illumin;
} SHTC3;
/*typedef struct
{
	uint8_t data[4];
	uint8_t lendata;
} */

extern EventGroupHandle_t mqtt_event_group;
//extern EventGroupHandle_t mqtt_event_group_read;

/*
  Name  : CRC-8
  Poly  : 0x31    x^8 + x^5 + x^4 + 1
  Init  : 0xFF
  Revert: false
  XorOut: 0x00
  Check : 0xF7 ("123456789")
  MaxLen: 15 байт(127 бит) - обнаружение
    одинарных, двойных, тройных и всех нечетных ошибок
*/
static uint8_t crc_8(uint8_t *pcBlock,  int len)
{
	uint8_t crc = 0xFF;
	//static const char *TAG = "CRC_TASK";
	//ESP_LOGI(TAG, "data %d crc %d", pcBlock, crc);
	int i=0;

	for (int k=0; k<len; k++)
	{
		crc ^= *(pcBlock+k);
		for (i = 0; i < 8; i++)
			crc = crc & 0x80 ? (crc << 1) ^ 0x31 : crc << 1;
	}
	//ESP_LOGI(TAG, "data %d crc %d", *pcBlock, crc);
	return crc;
}


static void tx_uart_task(void *arg)
{
	//uint8_t data;
	uint8_t data_byte, crc_data, data_massive[2];
	BaseType_t Status=pdPASS;
	static const char *TX_TASK_TAG = "TX_TASK";
	//esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);
	gpio_set_level(LED_GPIO, 0);
	while (1) {
		Status=xQueueReceive(Command_queue, &data_byte, 100);
		if(Status==pdPASS)
		{
			//ESP_LOGI(TX_TASK_TAG, "tx_task: %d", data_byte);
			crc_data=crc_8(&data_byte,1);
			data_massive[0]=data_byte;
			data_massive[1]=crc_data;
			uart_sendData(TX_TASK_TAG, data_massive,sizeof(data_massive));
			gpio_set_level(LED_GPIO, 1);
		}
		gpio_set_level(LED_GPIO, 0);
		vTaskDelay(250 / portTICK_PERIOD_MS);
		//gpio_set_level(LED_GPIO, 0);
	}
}
static void rx_uart_task(void *arg) 				//parser input data from stm32 by uart
{
	uint8_t crc_data=0x00;
	static const char *RX_TASK_TAG = "RX_TASK";
	esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
	uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE + 1);  //receive data buffer
	uint8_t str_dead[]={0xAD,0xDE};
	SHTC3 rx_data;
	while (1) {

		const int rxBytes = uart_read_bytes(UART_NUM_1, data, RX_BUF_SIZE, 1000 / portTICK_PERIOD_MS);
		//data[rxBytes] = 0;
		if(rxBytes>0)
		{
			ESP_LOGI(RX_TASK_TAG, "Read bytes: '%d'", rxBytes);
			switch(data[0])
			{
				case(0x1F):
				{
					//ESP_LOGI(RX_TASK_TAG, "Temperature data ");
					for(int i=0; i<rxBytes;i++)
					{
						*((uint8_t*)&rx_data.temperature+i)=*(data+(i+1));
					}
					for(int i=1; i<rxBytes;i++)
					{

						*((uint8_t*)&rx_data.humidity+i)=*(data+sizeof(float)+(i+1));
					}
					//ESP_LOGI(RX_TASK_TAG, "Read %d bytes:  '%.2f' '%.2f'", rxBytes,rx_data.temperature, rx_data.humidity);
					xQueueSend(Float_queue,&rx_data, 100);
					break;
				}
				case(0xAF):
				{
					//ESP_LOGI(RX_TASK_TAG, "Brigthnes data: '%d'", data[1]);
					crc_data=crc_8(data,rxBytes-1);
					ESP_LOGI(RX_TASK_TAG, "Read %d bytes %d", rxBytes, crc_data);
					if(crc_data==data[rxBytes-1])
					{
						//ESP_LOGI(RX_TASK_TAG, "Temperature data ");
						xQueueSend(Data_queue,&data[1], 100);
					}
					break;
				}
			}
		}
	}
	free(data);
}
static void status_task(void *arg)
{
	static const char *status_task = "status_TASK";
	uint16_t  data_event_read=0x00, BitstoWait=0x3FFF;
	uint8_t status_mqtt=0x00;
	/*uint8_t status_disc=0x01, status_connec=0x02;
	uint8_t status_sub=0x03, status_unsub=0x04;
	uint8_t status_pub=0x06, status_data = 0x08, status_error=0x09;*/


	while(1)
	{
		data_event_read = xEventGroupWaitBits(mqtt_event_group,BitstoWait, pdTRUE, pdFALSE, portMAX_DELAY);
		ESP_LOGI(status_task, "number event %d", data_event_read);
		//xQueueSend(Command_queue,&data_rx, 100);
		if((data_event_read & MQTT_EVT_PUBLIC_ERROR) && (data_event_read & MQTT_EVT_READ_ERROR))
		{
			status_mqtt = 0x09;
			ESP_LOGI(status_task, "status event %d", status_mqtt);
		}
		else
		{
			if(data_event_read & MQTT_EVT_PUBLIC_CONNECTED)
			{
				status_mqtt=0x02;
				ESP_LOGI(status_task, "status event %d", status_mqtt);
			}
			if((data_event_read & MQTT_EVT_PUBLIC_DISCONNECTED) &&  (data_event_read & MQTT_EVT_READ_DISCONNECTED))
			{
				status_mqtt=0x01;
				ESP_LOGI(status_task, "status event %d",status_mqtt);
			}
			if(data_event_read & MQTT_EVT_READ_SUBSCRIBED)
			{
				status_mqtt=0x03;
				ESP_LOGI(status_task, "status event %d", status_mqtt);
			}
			if(data_event_read & MQTT_EVT_READ_UNSUBSCRIBED)
			{
				status_mqtt=0x04;
				ESP_LOGI(status_task, "status event %d", status_mqtt);
			}
			if(data_event_read & MQTT_EVT_PUBLIC_PUBLISHED)
			{
				status_mqtt=0x06;
				ESP_LOGI(status_task, "status event %d", status_mqtt);
			}
			if(data_event_read & MQTT_EVT_READ_DATA)
			{
				status_mqtt=0x08;
				ESP_LOGI(status_task, "status event %d", status_mqtt);
			}
		}
		xQueueSend(Command_queue,&status_mqtt, 100);
		//vTaskDelay(10000 / portTICK_PERIOD_MS);
	}

}
static void mqtt_hum_task(void *arg)
{
	int i=0,  msg_id=0;
	uint8_t brigth=0, brigth_prev=0;
	uint8_t data_command=0x1F;
	BaseType_t Status=pdPASS;
	esp_mqtt_client_handle_t client;
	float temperature_prev = 0.0, humidity_prev = 0.0;
	static const char *mqtt_task = "mqtt_TASK";
	SHTC3 mqtt_data;
	char str01[30];
	char str02[30];
	char str03[30];

	//esp_log_level_set(mqtt_task, ESP_LOG_INFO);
	client=mqtt_start();
	//xQueueReceive(mqtt_queue, &client, 100);
	//ESP_LOGI(mqtt_task, "mqtt_start");
	while (1) {
		if ((i%2)==0)
		{
			data_command=0x1F;
			xQueueSend(Command_queue,&data_command, 100);
			ESP_LOGI(mqtt_task, "data_command: %d ", data_command);
			if (uxQueueMessagesWaiting( Float_queue )!=0)
			{
				xQueueReceive(Float_queue,&mqtt_data, 100);
				ESP_LOGI(mqtt_task, "Read_float_mqtt: '%.2f' '%.2f' ", mqtt_data.temperature, mqtt_data.humidity);
				if((mqtt_data.temperature<120) && ((mqtt_data.temperature>(-38))))
				{
					//ESP_LOGI(mqtt_task, "prev: %f data: %f ", temperature_prev, mqtt_data.temperature);
					sprintf(str01, "%.2f",mqtt_data.temperature);
					msg_id = esp_mqtt_client_publish(client, "gb_iot/2544_DSP/temperature", str01, 0, 1, 0);
					//ESP_LOGI(mqtt_task, "prev: %f data_rx: %f ", humidity_prev, mqtt_data.humidity);
					sprintf(str02, "%.2f",mqtt_data.humidity);
					msg_id = esp_mqtt_client_publish(client, "gb_iot/2544_DSP/humidity", str02, 0, 1, 0);
					/*if(fabs(temperature_prev-mqtt_data.temperature)>0.75)
					{
						//ESP_LOGI(mqtt_task, "prev: %f data: %f ", temperature_prev, mqtt_data.temperature);
						//sprintf(str01, "%.2f",mqtt_data.temperature);
						//msg_id = esp_mqtt_client_publish(client, "gb_iot/2544_DSP/temperature", str01, 0, 1, 0);
						temperature_prev = mqtt_data.temperature;
					}
					if(fabs(humidity_prev-mqtt_data.humidity)>0.75)
					{
						//ESP_LOGI(mqtt_task, "prev: %f data_rx: %f ", humidity_prev, mqtt_data.humidity);
						//sprintf(str02, "%.2f",mqtt_data.humidity);
						//msg_id = esp_mqtt_client_publish(client, "gb_iot/2544_DSP/humidity", str02, 0, 1, 0);
						humidity_prev= mqtt_data.humidity;
					}*/
				}
				/*else
				{

				}*/
			}
		}
		else
		{
			data_command=0xAF;
			xQueueSend(Command_queue,&data_command, 100);
			ESP_LOGI(mqtt_task, "data_command: %d ", data_command);
			if (uxQueueMessagesWaiting( Data_queue )!=0)
			{
				xQueueReceive(Data_queue,&brigth, 100);
				//ESP_LOGI(mqtt_task, "Read_illum_mqtt: '%d' ", brigth);
				//ESP_LOGI(mqtt_task, "Read_illum_mqtt: '%d' ", data_event);
				sprintf(str03, "%d",brigth);
				msg_id = esp_mqtt_client_publish(client, "gb_iot/2544_DSP/brigth", str03, 0, 1, 0);
				brigth_prev=brigth;

			}
		}
		i++;
		vTaskDelay(10000 / portTICK_PERIOD_MS);
	}
}


void app_main(void)
{

	//const char *app_main = "APP_MAIN";
	Command_queue = xQueueCreate(1, sizeof(uint8_t));
	Float_queue = xQueueCreate(1, sizeof(SHTC3));
	Data_queue = xQueueCreate(1, sizeof(uint8_t));
	uart_init();

	gpio_reset_pin(CONFIG_LED_GPIO);
	gpio_set_direction(CONFIG_LED_GPIO, GPIO_MODE_OUTPUT);
	gpio_set_level(CONFIG_LED_GPIO, 0);
	//Initialize NVS
	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
	{
		ret = nvs_flash_erase();
		ESP_LOGI(TAG, "nvs_flash_erase: 0x%04x", ret);
		ret = nvs_flash_init();
		ESP_LOGI(TAG, "nvs_flash_init: 0x%04x", ret);
	}
	ESP_LOGI(TAG, "nvs_flash_init: 0x%04x", ret);

	ret = esp_netif_init();
	ESP_LOGI(TAG, "esp_netif_init: %d", ret);
	ret = esp_event_loop_create_default();
	ESP_LOGI(TAG, "esp_event_loop_create_default: %d", ret);
	ret = wifi_init_sta();
	ESP_LOGI(TAG, "wifi_init_sta: %d", ret);
	xTaskCreate(rx_uart_task, "uart_rx_task", 1024 * 2, NULL, configMAX_PRIORITIES - 10, NULL);
	xTaskCreate(tx_uart_task, "uart_tx_task", 1024 * 2, NULL, configMAX_PRIORITIES - 10, NULL);
	xTaskCreate(mqtt_hum_task, "mqtt_hum_task", 1024 * 2, NULL, configMAX_PRIORITIES - 5, NULL);
	xTaskCreate(status_task, "status_task", 1024 * 2, NULL, configMAX_PRIORITIES - 5, NULL);
	//xTaskCreate(mqtt_hum_task, "mqtt_hum_task", 1024 * 2, NULL, configMAX_PRIORITIES - 5, NULL);
	while(1)
	{


		vTaskDelay(10000 / portTICK_PERIOD_MS);
	}
}
