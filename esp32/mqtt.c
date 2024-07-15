/*
 * mqtt.c
 *
 *  Created on: 4 янв. 2024 г.
 *      Author: Intel
 */

#include "mqtt.h"
//#include "secret.h"

//---------------------Server certificate--------------//
extern const uint8_t server_cert_pem_start[] asm("_binary_rootCA_crt_start");
extern const uint8_t server_cert_pem_end[] asm("_binary_rootCA_crt_end");
//-----------------------------------------------------//

//-------------------Certificate for publicate data----------------//
extern const uint8_t device_cert_pem_start[] asm("_binary_cert_devices_pem_start");
extern const uint8_t device_cert_pem_end[] asm("_binary_cert_devices_pem_end");
extern const uint8_t device_key_pem_start[] asm("_binary_key_devices_pem_start");
extern const uint8_t device_key_pem_end[] asm("_binary_key_devices_pem_end");
//-----------------------------------------------------------------//

//-------------------Certificate for receive data----------------//
extern const uint8_t registries_cert_pem_start[] asm("_binary_cert_registries_pem_start");
extern const uint8_t registries_cert_pem_end[] asm("_binary_cert_registries_pem_end");
extern const uint8_t registries_key_pem_start[] asm("_binary_key_registries_pem_start");
extern const uint8_t registries_key_pem_end[] asm("_binary_key_registries_pem_end");
//-----------------------------------------------------------------//


//-------------------------------------------------------------
static const char *TAG = "mqtt";
EventGroupHandle_t mqtt_event_group;
//EventGroupHandle_t mqtt_event_group_read;
//-------------------------------------------------------------

/*#define MQTT_EVT_CONNECTED 			BIT0
#define MQTT_EVT_SUBSCRIBED 			BIT1
#define MQTT_EVT_DISCONNECTED  		BIT2
#define MQTT_EVT_UNSUBSCRIBED  		BIT3
#define MQTT_EVT_DATA					BIT4*/

//-------------------------------------------------------------
//-------------------------------------------------------------
static void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0) {
      ESP_LOGE(TAG, "Last error %s: %d", message, error_code);
    }
}
//-------------------------------------------------------------

/*
 * @brief Event handler registered to receive MQTT events
 *
 *  This function is called by the MQTT client event loop.
 *
 * @param handler_args user data registered to the event.
 * @param base Event base for the handler(always MQTT Base in this example).
 * @param event_id The id for the received event.
 * @param event_data The data for the event, esp_mqtt_event_handle_t.
 */
static void mqtt_event_handler_publicate(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
 /// ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%ud", base, event_id);
  esp_mqtt_event_handle_t event = event_data;
  esp_mqtt_client_handle_t client = event->client;
  int msg_id;
  switch ((esp_mqtt_event_id_t)event_id) {
  case MQTT_EVENT_CONNECTED:
    xEventGroupSetBits(mqtt_event_group, MQTT_EVT_PUBLIC_CONNECTED);
    xEventGroupClearBits(mqtt_event_group, MQTT_EVT_PUBLIC_DISCONNECTED);
    xEventGroupClearBits(mqtt_event_group, MQTT_EVT_PUBLIC_ERROR);
    ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
    break;
  case MQTT_EVENT_DISCONNECTED:
    xEventGroupSetBits(mqtt_event_group, MQTT_EVT_PUBLIC_DISCONNECTED);
    xEventGroupClearBits(mqtt_event_group, MQTT_EVT_PUBLIC_CONNECTED);
    xEventGroupClearBits(mqtt_event_group, MQTT_EVT_PUBLIC_ERROR);
    ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
    break;
  case MQTT_EVENT_PUBLISHED:
    xEventGroupSetBits(mqtt_event_group, MQTT_EVT_PUBLIC_PUBLISHED);
    xEventGroupClearBits(mqtt_event_group, MQTT_EVT_PUBLIC_ERROR);
    ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED");
    break;
  case MQTT_EVENT_ERROR:
	xEventGroupSetBits(mqtt_event_group, MQTT_EVT_PUBLIC_ERROR);
    ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
    if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
      log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
      log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
      log_error_if_nonzero("captured as transport's socket errno",  event->error_handle->esp_transport_sock_errno);
      ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));
    }
    break;
  default:
    ESP_LOGI(TAG, "Other event id:%d", event->event_id);
    break;
  }
}
/*
 * @brief Event handler registered to receive MQTT events
 *
 *  This function is called by the MQTT client event loop.
 *
 * @param handler_args user data registered to the event.
 * @param base Event base for the handler(always MQTT Base in this example).
 * @param event_id The id for the received event.
 * @param event_data The data for the event, esp_mqtt_event_handle_t.
 */
static void mqtt_event_handler_readdata(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
 /// ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%ud", base, event_id);
  esp_mqtt_event_handle_t event = event_data;
  char str_subcribe_temp[]="gb_iot/1863_KAO/smarthome/temp";
  int num_data;
  esp_mqtt_client_handle_t client = event->client;
  int msg_id;
  switch ((esp_mqtt_event_id_t)event_id) {
  case MQTT_EVENT_CONNECTED:
    xEventGroupSetBits(mqtt_event_group, MQTT_EVT_READ_CONNECTED);
    xEventGroupClearBits(mqtt_event_group, MQTT_EVT_READ_DISCONNECTED);
    xEventGroupClearBits(mqtt_event_group, MQTT_EVT_READ_ERROR);
    ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED_read");
    char str01[30];
    sprintf(str01, "%d",0);
  //  msg_id = esp_mqtt_client_subscribe(client, str_subcribe_temp, 0);
    msg_id = esp_mqtt_client_subscribe(client, str_subcribe_temp, 0);
   // msg_id = esp_mqtt_client_subscribe(client, "gb_iot/2544_DSP/diode", 0);
    ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
    break;
  case MQTT_EVENT_DISCONNECTED:
    xEventGroupSetBits(mqtt_event_group, MQTT_EVT_READ_DISCONNECTED);
    xEventGroupClearBits(mqtt_event_group, MQTT_EVT_READ_CONNECTED);
    xEventGroupClearBits(mqtt_event_group, MQTT_EVT_READ_ERROR);
    ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED_read");
    break;
  case MQTT_EVENT_SUBSCRIBED:
    xEventGroupSetBits(mqtt_event_group, MQTT_EVT_READ_SUBSCRIBED);
    xEventGroupClearBits(mqtt_event_group, MQTT_EVT_READ_UNSUBSCRIBED);
    xEventGroupClearBits(mqtt_event_group, MQTT_EVT_READ_ERROR);
    ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED_read, msg_id=%d", event->msg_id);
    break;
  case MQTT_EVENT_UNSUBSCRIBED:
   // xEventGroupSetBits(mqtt_state_event_group, MQTT_EVT_READ_UNSUBSCRIBED);
   // xEventGroupClearBits(mqtt_state_event_group, MQTT_EVT_READ_SUBSCRIBED);
   // ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
    break;
  case MQTT_EVENT_DATA:
	xEventGroupSetBits(mqtt_event_group, MQTT_EVT_READ_DATA);
	xEventGroupClearBits(mqtt_event_group, MQTT_EVT_READ_ERROR);
	 TAG="mqtt_data";
	 ESP_LOGI(TAG, "MQTT_EVENT_DATA_read");
	// ESP_LOGI(TAG,"event_topic=:%s", event->topic);
	 //ESP_LOGI(TAG,"event_data=: %s", event->data);
	 num_data=atoi(event->data);
	 ESP_LOGI(TAG,"event_data_num=%d\n", num_data);
    break;
  case MQTT_EVENT_ERROR:
	xEventGroupSetBits(mqtt_event_group, MQTT_EVT_READ_ERROR);
    ESP_LOGI(TAG, "MQTT_EVENT_ERROR_read");
    if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
      log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
      log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
      log_error_if_nonzero("captured as transport's socket errno",  event->error_handle->esp_transport_sock_errno);
      ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));
    }
    break;
  default:
    ESP_LOGI(TAG, "Other event id:%d", event->event_id);
    break;
  }
}


esp_mqtt_client_handle_t mqtt_start(void)
//void mqtt_start(void)
{
	esp_err_t ret=ESP_OK, ret1=ESP_OK, ret2=ESP_OK, ret3=ESP_OK;

	//----------Client of publicate data-------//
	esp_mqtt_client_config_t mqtt_cfg = {
		.broker.address.uri = CONFIG_BROKER_URL,//"mqtts://mqtt.cloud.yandex.net:8883",
		.broker.verification.certificate = (const char *)server_cert_pem_start,
		.credentials = {
			.authentication = {
			  .certificate = (const char *)device_cert_pem_start,
			  .key = (const char *)device_key_pem_start,
			},
		}
	};
	//-----------------------------------------//

	//----------Client of receive data---------//
	esp_mqtt_client_config_t mqtt_cfg1 = {
			.broker.address.uri = CONFIG_BROKER_URL,//"mqtts://mqtt.cloud.yandex.net:8883",
			.broker.verification.certificate = (const char *)server_cert_pem_start,
			.credentials = {
				.authentication = {
				  .certificate = (const char *)registries_cert_pem_start,
				  .key = (const char *)registries_key_pem_start,
				},
			}
		};
	//-----------------------------------------//

	esp_mqtt_client_handle_t client_publicate = esp_mqtt_client_init(&mqtt_cfg);
	esp_mqtt_client_handle_t client_readdata = esp_mqtt_client_init(&mqtt_cfg1);

	mqtt_event_group = xEventGroupCreate();
	//mqtt_event_group_read = xEventGroupCreate();

	ret = esp_mqtt_client_register_event(client_publicate , ESP_EVENT_ANY_ID, mqtt_event_handler_publicate, NULL);
	ret1 = esp_mqtt_client_start(client_publicate);

	if(ret!=ESP_OK)
	{
		TAG="register_error";
		ESP_LOGI(TAG, "Other event id:%d", ret);
	}
	if(ret1!=ESP_OK)
	{
		TAG="start_client_error";
		ESP_LOGI(TAG, "Other event id:%d", ret1);
	}
	ret2 = esp_mqtt_client_register_event(client_readdata , ESP_EVENT_ANY_ID, mqtt_event_handler_readdata, NULL);
	ret3 = esp_mqtt_client_start(client_readdata);
	if(ret2!=ESP_OK)
	{
		TAG="register1_error";
		ESP_LOGI(TAG, "Other event id:%d", ret2);
	}
	if(ret3!=ESP_OK)
	{
		TAG="start_client1_error";
		ESP_LOGI(TAG, "Other event id:%d", ret3);
	}
	return client_publicate;

}
/*
static void mqtt_disconnect(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
  gpio_set_level(CONFIG_LED_GPIO, 0);
  ESP_LOGI(TAG, "Wi-Fi disconnected, trying to reconnect...");
  esp_err_t err = esp_wifi_connect();
  if (err == ESP_ERR_WIFI_NOT_STARTED) {
      return;
  }
  ESP_LOGI(TAG, "esp_wifi_connect() : %d", err);
}*/
