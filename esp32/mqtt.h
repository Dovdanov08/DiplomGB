/*
 * mqtt.h
 *
 *  Created on: 4 янв. 2024 г.
 *      Author: Intel
 */

#ifndef MAIN_MQTT_H_
#define MAIN_MQTT_H_

#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "mqtt_client.h"
#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "main.h"
//-------------------------------------------------------------

esp_mqtt_client_handle_t mqtt_start(void);
//void mqtt_start(void);
//void mqtt_start(void);
//-------------------------------------------------------------


#define MQTT_EVT_PUBLIC_CONNECTED 			BIT0
#define MQTT_EVT_PUBLIC_DISCONNECTED 		BIT1
#define MQTT_EVT_PUBLIC_PUBLISHED 			BIT2
#define MQTT_EVT_PUBLIC_ERROR 				BIT3

#define MQTT_EVT_READ_CONNECTED 				BIT4
#define MQTT_EVT_READ_DISCONNECTED 			BIT5
#define MQTT_EVT_READ_SUBSCRIBED				BIT6
#define MQTT_EVT_READ_UNSUBSCRIBED 			BIT7
#define MQTT_EVT_READ_DATA 					BIT8
#define MQTT_EVT_READ_ERROR 					BIT9

/*#define MQTT_EVT_CONNECTED 				BIT0
#define MQTT_EVT_SUBSCRIBED 				BIT1
#define MQTT_EVT_DISCONNECTED  			BIT2
#define MQTT_EVT_UNSUBSCRIBED  			BIT3
#define MQTT_EVT_DATA						BIT4
#define MQTT_EVT_ERROR						BIT5

#define MQTT_EVT_CONNECTED 				BIT6
#define MQTT_EVT_SUBSCRIBED 				BIT7
#define MQTT_EVT_DISCONNECTED  			BIT2
#define MQTT_EVT_UNSUBSCRIBED  			BIT3
#define MQTT_EVT_DATA						BIT4
#define MQTT_EVT_ERROR						BIT5*/




#endif /* MAIN_MQTT_H_ */
